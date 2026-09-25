using System.Diagnostics;
using System.Runtime.InteropServices;

namespace ViperCore
{
    /// <summary>
    /// Closed-loop joint controller for <see cref="ViperDevice"/>: a periodic
    /// loop that reads the arm, evaluates the trajectory, runs the controller
    /// and writes the goals.
    ///
    /// <para><b>It is the only thing that talks to the bus while running.</b>
    /// Each tick costs two transactions — one sync read, one sync write —
    /// regardless of joint count. Anyone else who needs the arm's state reads
    /// <see cref="ViperDevice.LatestState"/>, which is the snapshot this loop
    /// publishes and touches no hardware. Two callers issuing transactions on
    /// one serial port read each other's replies.</para>
    ///
    /// <para><b>The operating mode follows the controller.</b>
    /// <see cref="SetController"/> applies <see cref="IController.RequiredMode"/>,
    /// which needs torque off (EEPROM) and therefore has to happen before the
    /// motion starts, not during it.</para>
    ///
    /// <para><b>Timing.</b> The loop owns a dedicated thread rather than a
    /// <c>System.Threading.Timer</c>: a pool timer inherits Windows' ~15 ms
    /// scheduling granularity, which is coarser than the period itself at any
    /// rate worth running. The thread raises the system timer resolution while
    /// it runs and spins out the last stretch of each period. What it actually
    /// achieved is reported in <see cref="ActualRateHz"/> — a requested period
    /// the bus cannot sustain is silently the requested period no longer, and
    /// that shows up as motion in visible steps.</para>
    /// </summary>
    public sealed class JointController : IDisposable
    {
        // Windows' default scheduler granularity is ~15.6 ms, so Thread.Sleep(1)
        // really sleeps about 15. Asking for 1 ms makes the coarse part of the
        // wait usable and leaves only the last stretch to the spin.
        [DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
        private static extern uint TimeBeginPeriod(uint ms);

        [DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
        private static extern uint TimeEndPeriod(uint ms);

        /// <summary>Above this much time left in the period, sleep; below it, spin.</summary>
        private const double SpinThresholdSeconds = 0.003;

        private readonly ViperDevice _device;
        private readonly double      _period;

        private IController? _ctrl;
        private ITrajectory? _traj;

        // The ceiling the installed controller declared, or null when the mode
        // does not enforce one. Set in SetController, read in the tick.
        private double[]? _outputLimit;
        private bool      _limitReported;

        private Thread?       _thread;
        private volatile bool _stop;

        private double[] _qf = [];
        private double   _tf;
        private bool     _completed;
        private bool     _firstTick;
        private double   _tPrevTick;

        /// <summary>
        /// Loop time at which the trajectory was initialised. Everything the
        /// trajectory and the controller see is measured from here.
        /// </summary>
        private double _tOffset;

        // Every per-tick buffer is allocated once, in MoveTo. At 1 kHz a fresh
        // set each tick would be six thousand short-lived arrays a second, and
        // the GC pauses land inside the control loop.
        private double[]      _qpp    = [];
        private double[]      _qd     = [];
        private double[]      _qpd    = [];
        private double[]      _qppd   = [];
        private double[]      _output = [];
        private ControlInput? _input;

        // Acceleration is differentiated from the MEASURED velocity, so this is
        // one differentiation of a clean signal rather than two of a quantised
        // position. The first-order filter keeps the step-to-step noise of the
        // velocity register out of the result.
        private double[] _qpPrev  = [];
        private double[] _qppFilt = [];

        /// <summary>Corner of the acceleration filter, rad/s. Zero disables it.</summary>
        public double AccelerationFilter { get; set; } = 30.0;

        /// <summary>
        /// Seconds to keep ticking after the trajectory ends, holding its final
        /// point. Zero stops the instant the reference arrives.
        ///
        /// <para><b>The reference arriving is not the arm arriving.</b> A
        /// position servo trails its setpoint by roughly velocity over gain, so
        /// at <c>tf</c> the arm is still short by whatever it was lagging. Cut
        /// the loop there and the motion reports completed with the arm degrees
        /// away, and the tracking error never gets the chance to close.</para>
        ///
        /// <para>It also separates the two reasons an error stays open: over
        /// this window a lag decays to zero, while a load the joint cannot hold
        /// — gravity, stiction — settles on a constant offset and stays there.
        /// The value it settles at is that load, measured.</para>
        /// </summary>
        public double SettleTime { get; set; } = 0.5;

        /// <summary>Ticks executed in the current motion, failed reads included.</summary>
        public long TickCount { get; private set; }

        /// <summary>
        /// Ticks of the current motion that failed to read or write. A bus
        /// dropping packets advances the trajectory while the arm stops getting
        /// setpoints, which deforms the tracking error with nothing to show for
        /// it — so it is counted rather than only reported one at a time.
        /// </summary>
        public long FaultCount { get; private set; }

        public bool IsRunning   => _thread is { IsAlive: true };
        public bool IsCompleted => _completed;

        /// <summary>Period this loop was asked for, in seconds.</summary>
        public double RequestedPeriod => _period;

        /// <summary>
        /// Rate the loop is actually achieving, in Hz, smoothed over recent
        /// ticks. Zero before the first two ticks of a motion.
        ///
        /// <para>Worth reading rather than assuming: the requested period is a
        /// floor, and two serial transactions over USB do not always fit inside
        /// it. When they do not, the loop falls back to the rate the bus allows
        /// and the setpoints it streams get further apart.</para>
        /// </summary>
        public double ActualRateHz { get; private set; }

        /// <summary>
        /// Longest gap between two consecutive ticks of the current motion, in
        /// milliseconds. The worst case is what a controller has to survive, and
        /// it is not visible in the average.
        /// </summary>
        public double WorstTickMs { get; private set; }

        /// <summary>Raised when a tick fails to read or write. Fires on the loop thread.</summary>
        public event Action<string>? Fault;

        /// <summary>
        /// Raised at the end of every tick with that tick's data, on the loop
        /// thread.
        ///
        /// <para><b>The argument is reused between ticks.</b> A handler must
        /// copy out whatever it needs before returning, and must return quickly:
        /// it runs inside the control period.</para>
        /// </summary>
        public event Action<ControlInput>? Sampled;

        /// <param name="sampleTime">Loop period in seconds (default 50 ms).</param>
        public JointController(ViperDevice device, double sampleTime = 0.05)
        {
            _device = device;
            _period = sampleTime;
        }

        /// <summary>
        /// Installs the controller and puts the motors in the mode it requires.
        /// Leaves torque OFF — <see cref="MoveTo"/> turns it back on — because
        /// the mode change has to pass through EEPROM and re-enabling early
        /// would let the arm jump to whatever stale goal the new mode's register
        /// holds.
        ///
        /// <para>It is also where <see cref="IController.OutputLimit"/> reaches the
        /// motors, and that is not a free choice of place: <c>Current Limit</c> is
        /// an EEPROM register, so writing it needs torque off, and the mode change
        /// has just turned it off anyway. Doing it anywhere else costs the arm a
        /// second fall.</para>
        /// </summary>
        public bool SetController(IController controller)
        {
            _ctrl          = controller;
            _limitReported = false;

            // Only kept when the mode actually enforces it. A position
            // controller's output is degrees, and clamping degrees against a
            // current ceiling would be nonsense.
            _outputLimit = controller.RequiredMode == OperatingMode.Current
                ? controller.OutputLimit
                : null;

            bool modeChanged = _device.Mode != controller.RequiredMode;

            if (modeChanged && !_device.SetOperatingMode(controller.RequiredMode))
                return false;

            // If the mode did not change, torque may well be on, and SetCurrentLimit
            // turns it off to write EEPROM and leaves it off. That is a fall, so it
            // only happens when there is a limit to write — and MoveTo re-energises
            // through EnableTorqueHolding either way.
            if (_outputLimit != null && !_device.SetCurrentLimit(_outputLimit))
                return false;

            return true;
        }

        public void SetTrajectory(ITrajectory trajectory) => _traj = trajectory;

        /// <summary>
        /// Starts a motion to <paramref name="qf"/> (Dynamixel degrees) over
        /// <paramref name="tf"/> seconds. Enables torque before the first tick.
        /// </summary>
        public bool MoveTo(double[] qf, double tf)
        {
            Stop();

            if (qf.Length != _device.JointCount)
            {
                Fault?.Invoke($"MoveTo espera {_device.JointCount} valores, recibió {qf.Length}.");
                return false;
            }

            int n = _device.JointCount;

            _qf        = (double[])qf.Clone();
            _tf        = tf;
            _completed = false;
            _firstTick = true;
            _tPrevTick = 0.0;
            _tOffset   = 0.0;

            _qpPrev  = new double[n];
            _qppFilt = new double[n];
            _qpp     = new double[n];
            _qd      = new double[n];
            _qpd     = new double[n];
            _qppd    = new double[n];
            _output  = new double[n];
            _input   = new ControlInput();

            ActualRateHz = 0.0;
            WorstTickMs  = 0.0;
            TickCount    = 0;
            FaultCount   = 0;

            _ctrl?.Reset();

            // Torque comes on holding the present pose, never bare. A plain
            // EnableTorque would servo every motor to whatever stale Goal
            // Position its register holds — after any spell with torque off,
            // the pose from before the arm sagged — and it would do it in the
            // gap before the first tick writes a real command. With Profile
            // Velocity at 0 (no limit) that gap is a snap, not a drift.
            //
            // Current mode goes through the same door: there is no pose to hold
            // there, but there is a zero, and EnableTorqueHolding writes it before
            // energising. This used to be a bare EnableTorque, which energised the
            // arm against whatever Goal Current the register still held from a
            // previous run — the gap before the first tick, with an unknown
            // command in it, and the arm already sagging from the mode change.
            //
            // Velocity and PWM still do not: neither has a command that means
            // "stay", so energising there remains the open problem it is in
            // EmergencyStop.
            bool energised = _device.Mode is OperatingMode.Position or
                                             OperatingMode.ExtendedPosition or
                                             OperatingMode.CurrentPosition or
                                             OperatingMode.Current
                ? _device.EnableTorqueHolding()
                : _device.EnableTorque(true);

            if (!energised)
            {
                Fault?.Invoke(_device.LastError ?? "No se pudo habilitar el par.");
                return false;
            }

            _stop   = false;
            _thread = new Thread(RunLoop)
            {
                IsBackground = true,
                Name         = "ViperControlLoop",
                // Above normal, not highest: this loop spins, and starving the
                // UI thread it reports to helps nobody.
                Priority     = ThreadPriority.AboveNormal,
            };
            _thread.Start();
            return true;
        }

        /// <summary>
        /// Stops ticking. Leaves torque as it is, so the arm holds its last
        /// commanded pose rather than dropping. To stop and freeze deliberately
        /// use <see cref="ViperDevice.EmergencyStop"/>.
        /// </summary>
        public void Stop()
        {
            Thread? t = _thread;
            _stop   = true;
            _thread = null;

            // A tick that finished the trajectory calls Stop on the loop thread
            // itself; joining there would deadlock.
            if (t != null && t != Thread.CurrentThread && t.IsAlive)
                t.Join(500);
        }

        // ── The loop ─────────────────────────────────────────────────────────

        private void RunLoop()
        {
            TimeBeginPeriod(1);
            var clock = Stopwatch.StartNew();

            try
            {
                double tStart   = clock.Elapsed.TotalSeconds;
                double nextTick = tStart;
                double prevTick = double.NaN;

                while (!_stop)
                {
                    double now = clock.Elapsed.TotalSeconds;

                    if (!double.IsNaN(prevTick))
                    {
                        double spacing = now - prevTick;
                        WorstTickMs = Math.Max(WorstTickMs, spacing * 1000.0);

                        // Exponential mean over roughly the last twenty ticks,
                        // so the number settles quickly but does not chase a
                        // single late tick.
                        double hz = spacing > 0.0 ? 1.0 / spacing : 0.0;
                        ActualRateHz = ActualRateHz <= 0.0
                            ? hz
                            : ActualRateHz + 0.1 * (hz - ActualRateHz);
                    }
                    prevTick = now;

                    Tick(now - tStart);

                    if (_stop) break;

                    nextTick += _period;
                    double after = clock.Elapsed.TotalSeconds;

                    // Behind schedule: resync instead of firing the backlog
                    // back to back. Bunched ticks would put transactions on the
                    // bus faster than it can carry them and buy nothing.
                    if (after >= nextTick) nextTick = after;
                    else WaitUntil(clock, nextTick);
                }
            }
            finally
            {
                TimeEndPeriod(1);
            }
        }

        private void WaitUntil(Stopwatch clock, double until)
        {
            while (!_stop)
            {
                double remaining = until - clock.Elapsed.TotalSeconds;
                if (remaining <= 0.0) return;

                if (remaining > SpinThresholdSeconds) Thread.Sleep(1);
                else Thread.SpinWait(100);
            }
        }

        private void Tick(double t)
        {
            TickCount++;

            JointState s = _device.ReadState();
            if (!s.Valid)
            {
                // A dropped read is not a reason to stop: the arm holds its last
                // goal and the next tick usually succeeds. Skipping keeps the
                // controller from working on stale measurements.
                RaiseFault(_device.LastError ?? "Lectura fallida.");
                return;
            }

            int n = s.Count;

            if (_firstTick)
            {
                // The trajectory starts from wherever the arm actually is, so
                // its first setpoint is a no-op and nothing jumps.
                //
                // Its clock starts here too, not when the loop did. The two are
                // the same tick when the first read succeeds — but a read that
                // fails is skipped, and without this offset the trajectory would
                // be initialised at the present pose and then immediately asked
                // for its value at a time that had already elapsed, which is a
                // step the arm has to chase.
                _traj?.Init(s.Position, _qf, _tf);
                Array.Copy(s.Velocity, _qpPrev, n);
                _tOffset   = t;
                _tPrevTick = t;
                _firstTick = false;
            }

            double dt = t - _tPrevTick;
            if (dt <= 0.0) dt = _period;
            _tPrevTick = t;

            double tTraj = t - _tOffset;

            // Acceleration from the measured velocity, low-passed.
            double lam = AccelerationFilter;
            for (int i = 0; i < n; i++)
            {
                double raw = (s.Velocity[i] - _qpPrev[i]) / dt;
                if (lam > 0.0)
                {
                    _qppFilt[i] += lam * (raw - _qppFilt[i]) * dt;
                    _qpp[i] = _qppFilt[i];
                }
                else
                {
                    _qpp[i] = raw;
                }
                _qpPrev[i] = s.Velocity[i];
            }

            _traj?.Evaluate(tTraj, _qd, _qpd, _qppd);

            ControlInput input = _input!;
            input.Q    = s.Position; input.Qp  = s.Velocity; input.Qpp = _qpp;
            input.Current = s.Current;
            input.Qd   = _qd;        input.Qpd = _qpd;       input.Qppd = _qppd;
            input.T    = tTraj;      input.Dt  = dt;

            if (_ctrl != null) _ctrl.Compute(input, _output);
            else               Array.Copy(_qd, _output, n);

            ClampOutput();

            if (!_device.WriteGoals(_output))
                RaiseFault(_device.LastError ?? "Escritura fallida.");

            Sampled?.Invoke(input);

            // The trajectory is done at tf; the arm is not. SettleTime keeps the
            // loop holding qf long enough for the tracking error to close, or to
            // show that it does not.
            if (!_completed && tTraj >= _tf + SettleTime)
            {
                _completed = true;
                _stop      = true;
            }
        }

        /// <summary>
        /// Bounds the controller's output against the ceiling it declared, and
        /// reports the first time the bound bites.
        ///
        /// <para>The motors enforce the same ceiling themselves, so this is not
        /// what keeps the arm safe — it is what makes a saturation <b>visible</b>.
        /// The firmware clips without a word, so a model that asks for three times
        /// what it should and a model that asks for exactly the limit look
        /// identical from here. One of those is a bug.</para>
        ///
        /// <para>NaN is clamped to zero and reported too. It cannot come from a
        /// correct model, but <c>Encode</c> would turn it into an arbitrary integer
        /// on its way to a motor, and "arbitrary" is not a current to send an arm.
        /// Zero is the one value that is always safe to write.</para>
        /// </summary>
        private void ClampOutput()
        {
            double[]? limit = _outputLimit;
            if (limit == null) return;

            for (int i = 0; i < _output!.Length && i < limit.Length; i++)
            {
                double value = _output[i];

                if (double.IsNaN(value) || double.IsInfinity(value))
                {
                    _output[i] = 0.0;
                    ReportClamp($"El controlador devolvió {value} en la articulación {i + 1}. " +
                                "Se escribió 0 en su lugar.");
                    continue;
                }

                double bound = Math.Abs(limit[i]);
                if (Math.Abs(value) <= bound) continue;

                _output[i] = value < 0.0 ? -bound : bound;
                ReportClamp($"La articulación {i + 1} pidió {value:F0} y su tope es " +
                            $"{bound:F0}. Se recortó; revise el modelo antes de seguir.");
            }
        }

        /// <summary>
        /// Once per motion, not once per tick: at 333 Hz a controller that
        /// saturates continuously would bury every other fault under thousands of
        /// copies of the same line.
        /// </summary>
        private void ReportClamp(string message)
        {
            if (_limitReported) return;

            _limitReported = true;
            RaiseFault(message);
        }

        private void RaiseFault(string message)
        {
            FaultCount++;
            Fault?.Invoke(message);
        }

        public void Dispose() => Stop();
    }
}
