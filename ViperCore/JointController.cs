namespace ViperCore
{
    /// <summary>
    /// Closed-loop joint controller for ViperDevice.
    /// Mirrors GeomagicCore's JointController: pluggable IController + ITrajectory,
    /// driven by a periodic timer. Instead of setTorques, applies SetJointAngles.
    /// </summary>
    public sealed class JointController : IDisposable
    {
        private readonly ViperDevice _device;
        private readonly double      _sampleTime;

        private IController? _ctrl;
        private ITrajectory? _traj;

        private System.Threading.Timer? _timer;
        private int  _tickRunning;   // Interlocked flag — prevents concurrent ticks

        private double[]  _qf        = [];
        private double    _tf;
        private DateTime  _tStart;
        private bool      _firstTick;
        private bool      _completed;

        public bool IsRunning   => _timer != null;
        public bool IsCompleted => _completed;

        /// <param name="sampleTime">Timer period in seconds (default 50 ms).</param>
        public JointController(ViperDevice device, double sampleTime = 0.05)
        {
            _device     = device;
            _sampleTime = sampleTime;
        }

        public void SetController(IController controller) => _ctrl = controller;
        public void SetTrajectory(ITrajectory trajectory) => _traj = trajectory;

        /// <summary>Starts a motion to qf (radians) completed in tf seconds.</summary>
        public void MoveTo(double[] qf, double tf)
        {
            Stop();

            _qf        = (double[])qf.Clone();
            _tf        = tf;
            _firstTick = true;
            _completed = false;
            _ctrl?.Reset();

            int periodMs = (int)(_sampleTime * 1000);
            _timer = new System.Threading.Timer(Tick, null, 0, periodMs);
        }

        public void Stop()
        {
            _timer?.Dispose();
            _timer = null;
        }

        private void Tick(object? _)
        {
            // Skip tick if a previous one is still executing (serial I/O takes ~24 ms).
            if (Interlocked.CompareExchange(ref _tickRunning, 1, 0) != 0) return;

            try
            {
                double[] q = _device.GetJointAngles();

                if (_firstTick)
                {
                    _tStart    = DateTime.UtcNow;
                    _traj?.Init(q, _qf, _tf);
                    _firstTick = false;
                }

                double t  = (DateTime.UtcNow - _tStart).TotalSeconds;
                int    n  = q.Length;

                double[] qd = new double[n];
                _traj?.Evaluate(t, qd);

                var input = new ControlInput
                {
                    Q    = q,
                    Qd   = qd,
                    Qpf  = new double[n],
                    Qppf = new double[n],
                    T    = t,
                    Dt   = _sampleTime
                };

                double[] output = new double[n];
                if (_ctrl != null)
                    _ctrl.Compute(input, output);
                else
                    Array.Copy(qd, output, n);

                _device.SetJointAngles(output);

                if (!_completed && t >= _tf)
                {
                    _completed = true;
                    Stop();
                }
            }
            finally
            {
                Interlocked.Exchange(ref _tickRunning, 0);
            }
        }

        public void Dispose() => Stop();
    }
}
