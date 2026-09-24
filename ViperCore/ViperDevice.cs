namespace ViperCore
{
    /// <summary>
    /// Connection to a Viper X-300S arm over Dynamixel Protocol 2.0.
    ///
    /// <para><b>Units.</b> Everything public is in <b>Dynamixel degrees</b>:
    /// 0–360 with 180° at the motor centre, which is what Dynamixel Wizard
    /// shows. Any conversion to signed joint angles belongs above this class.
    /// Velocities are deg/s and currents mA, both signed.</para>
    ///
    /// <para><b>Threading.</b> One serial port cannot serve two callers at once:
    /// concurrent transactions read each other's reply packets. Every operation
    /// here takes a lock, and the intended pattern is that only the control loop
    /// calls <see cref="ReadState"/> while everyone else reads
    /// <see cref="LatestState"/>, which is a cached copy and touches no
    /// hardware.</para>
    ///
    /// <para><b>Cost.</b> State comes back through one sync read of the ten
    /// contiguous bytes at 126 (current, velocity, position) and goals go out
    /// through one sync write — two transactions per tick regardless of the
    /// number of joints, instead of one per motor per register.</para>
    /// </summary>
    public sealed class ViperDevice : IDisposable
    {
        // ── Motor map ────────────────────────────────────────────────────────
        //
        //  Joint  │ ID(s)  │ Model
        //  ───────┼────────┼────────────
        //  q1     │  1     │ XM540-W270
        //  q2     │  2, 3  │ XM540-W270  (dual — 3 shadows 2)
        //  q3     │  4, 5  │ XM540-W270  (dual — 5 shadows 4)
        //  q4     │  6     │ XM540-W270
        //  q5     │  7     │ XM540-W270
        //  q6     │  8     │ XM430-W350
        //  gripper│  9     │ XM430-W350   — handled apart, see GripperId

        /// <summary>The six arm joints, one primary motor each.</summary>
        public static readonly byte[] JointMotorIds = [1, 2, 4, 6, 7, 8];

        /// <summary>Secondary motors of the dual joints. They shadow their primary.</summary>
        public static readonly byte[] ShadowMotorIds = [3, 5];

        /// <summary>Primary of each entry of <see cref="ShadowMotorIds"/>.</summary>
        public static readonly byte[] ShadowPrimaryIds = [2, 4];

        /// <summary>
        /// The gripper. Not a joint: it is driven as an end effector and is
        /// excluded from <see cref="ReadState"/> and <see cref="WriteGoals"/>.
        /// </summary>
        public const byte GripperId = 9;

        /// <summary>
        /// The eight motors that make up the arm itself. All of them are
        /// required: a missing one is a joint that cannot be read or driven.
        /// </summary>
        public static readonly byte[] ArmMotorIds = [1, 2, 3, 4, 5, 6, 7, 8];

        /// <summary>Everything on the bus when the arm is fully assembled.</summary>
        public static readonly byte[] AllMotorIds = [1, 2, 3, 4, 5, 6, 7, 8, 9];

        public int JointCount => JointMotorIds.Length;

        // ── State ────────────────────────────────────────────────────────────

        private readonly object _io = new();
        private int  _portNum = -1;
        private bool _connected;
        private JointState _latest = JointState.Empty(JointMotorIds.Length);
        private byte[] _present = [];

        public bool    IsConnected => _connected;
        public string? LastError   { get; private set; }

        /// <summary>
        /// Whether the gripper answered at connect. It is an end effector, not a
        /// joint, and the arm is perfectly usable without it — so it is detected
        /// rather than required.
        /// </summary>
        public bool HasGripper { get; private set; }

        /// <summary>
        /// The motors that actually answered at connect. Every bus-wide
        /// operation walks this rather than <see cref="AllMotorIds"/>, so a
        /// detached gripper does not turn each of them into a failure.
        /// </summary>
        public IReadOnlyList<byte> PresentMotorIds => _present;

        /// <summary>
        /// Operating mode currently configured, as last set through
        /// <see cref="SetOperatingMode"/> or read at connect.
        /// </summary>
        public OperatingMode Mode { get; private set; } = OperatingMode.Position;

        /// <summary>
        /// Last snapshot read by <see cref="ReadState"/>. Safe to call from any
        /// thread; touches no hardware. Returns a copy.
        /// </summary>
        public JointState LatestState
        {
            get { lock (_io) return _latest.Clone(); }
        }

        // ── Connection ───────────────────────────────────────────────────────

        public bool Connect(string portName, int baudRate = 1_000_000)
        {
            Disconnect();

            lock (_io)
            {
                _portNum = DynamixelSDK.PortHandler(portName);
                DynamixelSDK.PacketHandler();

                if (!DynamixelSDK.OpenPort(_portNum))
                    return Fail($"No se pudo abrir el puerto {portName}.");

                if (!DynamixelSDK.SetBaudRate(_portNum, baudRate))
                    return Fail($"No se pudo configurar baud rate {baudRate}.");

                // Every silent motor is collected before giving up, rather than
                // failing on the first: on a bring-up, learning about one dead
                // ID per attempt turns one diagnosis into eight.
                var silent = new List<byte>();
                foreach (byte id in ArmMotorIds)
                {
                    DynamixelSDK.Ping(_portNum, DynamixelSDK.PROTOCOL, id);
                    if (!LastTransactionOk(out _)) silent.Add(id);
                }

                if (silent.Count > 0)
                    return Fail($"No respondieron los motores con ID {string.Join(", ", silent)}. " +
                                "Revise el puerto, el baud rate y la alimentación del brazo.");

                // The gripper is optional. Detaching it is a normal thing to do
                // — it is an end effector — so its silence is recorded, not
                // treated as a broken arm.
                DynamixelSDK.Ping(_portNum, DynamixelSDK.PROTOCOL, GripperId);
                HasGripper = LastTransactionOk(out _);

                _present = HasGripper
                    ? [.. ArmMotorIds, GripperId]
                    : [.. ArmMotorIds];

                // Adopt whatever mode the motors are already in, so the first
                // command does not contradict the hardware.
                byte raw = DynamixelSDK.Read1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL,
                    JointMotorIds[0], ControlTable.OperatingMode);
                if (LastTransactionOk(out _))
                    Mode = (OperatingMode)raw;

                _connected = true;
                _latest    = JointState.Empty(JointMotorIds.Length);
                LastError  = null;
                return true;
            }
        }

        public void Disconnect()
        {
            lock (_io)
            {
                if (_portNum >= 0)
                {
                    DynamixelSDK.ClosePort(_portNum);
                    _portNum = -1;
                }
                _connected = false;
                _present    = [];
                HasGripper  = false;
            }
        }

        // ── Torque ───────────────────────────────────────────────────────────

        /// <summary>
        /// Enables or disables torque on one motor.
        ///
        /// <para><b>Disabling torque on an arm with no brakes drops it.</b> For
        /// stopping motion use <see cref="EmergencyStop"/>, which freezes the
        /// arm in place with torque still on. Torque off should be a deliberate
        /// act, ideally with the arm already at a rest pose.</para>
        /// </summary>
        public bool EnableTorque(byte id, bool enable)
        {
            lock (_io)
            {
                if (!Ready(out _)) return false;
                DynamixelSDK.Write1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                    ControlTable.TorqueEnable, enable ? (byte)1 : (byte)0);
                if (!LastTransactionOk(out string why))
                    return Fail($"Motor ID {id}: no se pudo {(enable ? "habilitar" : "deshabilitar")} el par ({why}).");
                return true;
            }
        }

        /// <summary>
        /// Enables or disables torque on every motor that answered at connect —
        /// the gripper included when it is attached.
        /// </summary>
        public bool EnableTorque(bool enable)
        {
            bool ok = true;
            foreach (byte id in _present)
                ok &= EnableTorque(id, enable);
            return ok;
        }

        /// <summary>
        /// Whether any motor currently holds torque. Null on a communication
        /// failure — the caller must not read that as "off".
        ///
        /// <para>True when <b>any</b> motor is energised, not all of them: a
        /// partly live arm is live, and reporting it as off is the answer that
        /// gets someone hurt.</para>
        /// </summary>
        public bool? ReadTorqueEnabled()
        {
            lock (_io)
            {
                if (!Ready(out _)) return null;

                bool any = false;
                foreach (byte id in _present)
                {
                    byte raw = DynamixelSDK.Read1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                        ControlTable.TorqueEnable);
                    if (!LastTransactionOk(out string why))
                    {
                        Fail($"Motor ID {id}: no se pudo leer el estado del par ({why}).");
                        return null;
                    }
                    if (raw != 0) any = true;
                }
                return any;
            }
        }

        /// <summary>
        /// Enables torque without moving the arm: reads the present position and
        /// writes it back as the goal first, so the motors take hold of the pose
        /// they are already in.
        ///
        /// <para>This is the only safe way to energise this arm. A bare
        /// <see cref="EnableTorque(bool)"/> makes every motor servo to whatever
        /// Goal Position the register happens to hold — after any spell with
        /// torque off, that is the pose from before the arm sagged, and the arm
        /// snaps to it at whatever Profile Velocity allows. Profile Velocity 0
        /// means <b>no limit</b>, not "stop".</para>
        ///
        /// <para><b>Only valid in the position modes</b>, for the same reason as
        /// <see cref="EmergencyStop"/>: the other modes ignore Goal Position, so
        /// there is no pose to take hold of. Fails explicitly there rather than
        /// energising the arm on an unknown goal.</para>
        /// </summary>
        public bool EnableTorqueHolding()
        {
            // The lock is reentrant, so ReadState and WriteGoals taking it again
            // is fine — and it keeps read-goal-enable atomic against the loop.
            lock (_io)
            {
                if (!Ready(out _)) return false;

                if (Mode is not (OperatingMode.Position or
                                 OperatingMode.ExtendedPosition or
                                 OperatingMode.CurrentPosition))
                {
                    return Fail("Habilitar el par sujetando la pose sólo está implementado " +
                                $"para los modos de posición; el modo actual es {Mode}.");
                }

                JointState s = ReadState();
                if (!s.Valid)
                    return Fail("No se pudo leer la posición actual para habilitar el par.");

                if (!WriteGoals(s.Position)) return false;

                return EnableTorque(true);
            }
        }

        // ── Operating mode ───────────────────────────────────────────────────

        /// <summary>
        /// Sets the operating mode of every joint motor (and the shadows).
        ///
        /// <para>Operating Mode is EEPROM, so this disables torque, writes, and
        /// leaves torque <b>off</b>. The caller decides when to re-enable it —
        /// re-enabling here would make the arm jump to whatever stale goal the
        /// new mode's register happens to hold.</para>
        /// </summary>
        public bool SetOperatingMode(OperatingMode mode)
        {
            lock (_io)
            {
                if (!Ready(out _)) return false;

                foreach (byte id in _present)
                {
                    DynamixelSDK.Write1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                        ControlTable.TorqueEnable, 0);
                    if (!LastTransactionOk(out string why))
                        return Fail($"Motor ID {id}: no se pudo apagar el par para cambiar de modo ({why}).");
                }

                foreach (byte id in _present)
                {
                    DynamixelSDK.Write1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                        ControlTable.OperatingMode, (byte)mode);
                    if (!LastTransactionOk(out string why))
                        return Fail($"Motor ID {id}: no se pudo escribir el modo de operación ({why}).");
                }

                Mode = mode;
                return true;
            }
        }

        // ── Profiles and limits ──────────────────────────────────────────────

        /// <summary>
        /// Profile velocity, in deg/s, applied to every joint. Caps how fast the
        /// motor slews towards a goal, which is a cheap guard against a large
        /// jump in command.
        ///
        /// <para><b>Zero means "no limit", not "stop".</b> That is the register's
        /// own convention and a classic way to get a surprise. Only an argument
        /// of exactly zero produces it: a small positive limit is rounded up to
        /// one unit rather than down into "no limit", which would turn the
        /// slowest request anyone can make into the fastest.</para>
        /// </summary>
        public bool SetProfileVelocity(double degPerSec)
        {
            uint units = ToProfileUnits(degPerSec, ControlTable.DegPerSecPerVelUnit);
            return WriteAllJoints4(ControlTable.ProfileVelocity, units, "profile velocity");
        }

        /// <summary>
        /// Profile acceleration, in deg/s². Zero means no limit, and a small
        /// positive value rounds up to one unit — same reasoning as
        /// <see cref="SetProfileVelocity"/>.
        /// </summary>
        public bool SetProfileAcceleration(double degPerSec2)
        {
            // Profile acceleration is 214.577 rev/min² per unit.
            const double degPerSec2PerUnit = 214.577 * 360.0 / 3600.0;
            uint units = ToProfileUnits(degPerSec2, degPerSec2PerUnit);
            return WriteAllJoints4(ControlTable.ProfileAccel, units, "profile acceleration");
        }

        /// <summary>
        /// Converts a profile limit to register units, keeping "no limit"
        /// reachable only by asking for it explicitly.
        /// </summary>
        private static uint ToProfileUnits(double value, double perUnit)
        {
            if (value <= 0.0) return 0;                       // asked for no limit
            return (uint)Math.Max(1, Math.Round(value / perUnit));
        }

        /// <summary>
        /// Goal current, in mA. In <see cref="OperatingMode.CurrentPosition"/>
        /// this is the cap the position controller may draw, which is what makes
        /// the arm hold its pose but yield when pushed.
        /// </summary>
        public bool SetGoalCurrent(double milliamps)
        {
            int units = (int)Math.Round(milliamps / ControlTable.MilliampsPerCurrentUnit);
            lock (_io)
            {
                if (!Ready(out _)) return false;
                foreach (byte id in JointMotorIds)
                {
                    DynamixelSDK.Write2ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                        ControlTable.GoalCurrent, (ushort)ControlTable.FromSigned16(units));
                    if (!LastTransactionOk(out string why))
                        return Fail($"Motor ID {id}: no se pudo escribir goal current ({why}).");
                }
                return true;
            }
        }

        /// <summary>
        /// Current limit, in mA. EEPROM: requires torque off, and has a finite
        /// number of write cycles. This is the absolute ceiling; for a runtime
        /// cap use <see cref="SetGoalCurrent"/>.
        /// </summary>
        public bool SetCurrentLimit(double milliamps)
        {
            uint units = (uint)Math.Max(0, Math.Round(milliamps / ControlTable.MilliampsPerCurrentUnit));
            lock (_io)
            {
                if (!Ready(out _)) return false;
                foreach (byte id in _present)
                {
                    DynamixelSDK.Write1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                        ControlTable.TorqueEnable, 0);
                    if (!LastTransactionOk(out string why))
                        return Fail($"Motor ID {id}: no se pudo apagar el par para el límite de corriente ({why}).");

                    DynamixelSDK.Write2ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                        ControlTable.CurrentLimit, (ushort)units);
                    if (!LastTransactionOk(out string why2))
                        return Fail($"Motor ID {id}: no se pudo escribir el límite de corriente ({why2}).");
                }
                return true;
            }
        }

        // ── Faults ───────────────────────────────────────────────────────────

        /// <summary>
        /// Reads Hardware Error Status from every motor. A motor that latches
        /// overload or overheating disables its own torque, and on an arm with
        /// no brakes that means it falls — so this is worth polling and
        /// surfacing rather than letting the drop look inexplicable.
        /// Returns null on a communication failure.
        /// </summary>
        public Dictionary<byte, HardwareError>? ReadHardwareErrors()
        {
            lock (_io)
            {
                if (!Ready(out _)) return null;

                var result = new Dictionary<byte, HardwareError>();
                foreach (byte id in _present)
                {
                    byte raw = DynamixelSDK.Read1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                        ControlTable.HardwareError);
                    if (!LastTransactionOk(out string why))
                    {
                        Fail($"Motor ID {id}: no se pudo leer el estado de error ({why}).");
                        return null;
                    }
                    result[id] = (HardwareError)raw;
                }
                return result;
            }
        }

        // ── Dual-motor coupling ──────────────────────────────────────────────

        /// <summary>
        /// Reads the Secondary ID and Drive Mode of the shadow motors, so the
        /// coupling can be checked instead of assumed. Returns null on a
        /// communication failure.
        ///
        /// <para>This is deliberately read-only. The correct Drive Mode depends
        /// on how this particular arm is assembled — the two motors of a joint
        /// are mounted facing each other, so one has to turn the other way — and
        /// writing it wrong makes them fight and overheat. Use
        /// <see cref="ConfigureCoupling"/> only after looking at what this
        /// returns.</para>
        /// </summary>
        public List<CouplingStatus>? VerifyCoupling()
        {
            lock (_io)
            {
                if (!Ready(out _)) return null;

                var list = new List<CouplingStatus>();
                for (int i = 0; i < ShadowMotorIds.Length; i++)
                {
                    byte id = ShadowMotorIds[i];

                    byte secondary = DynamixelSDK.Read1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL,
                        id, ControlTable.SecondaryId);
                    if (!LastTransactionOk(out string why))
                    {
                        Fail($"Motor ID {id}: no se pudo leer Secondary ID ({why}).");
                        return null;
                    }

                    byte drive = DynamixelSDK.Read1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL,
                        id, ControlTable.DriveMode);
                    if (!LastTransactionOk(out string why2))
                    {
                        Fail($"Motor ID {id}: no se pudo leer Drive Mode ({why2}).");
                        return null;
                    }

                    list.Add(new CouplingStatus(id, ShadowPrimaryIds[i], secondary, drive));
                }
                return list;
            }
        }

        /// <summary>
        /// Writes Secondary ID and Drive Mode on one shadow motor. EEPROM, so
        /// torque goes off and stays off.
        ///
        /// <para>Not called automatically anywhere. Getting <paramref name="reversed"/>
        /// wrong makes the two motors of a joint pull against each other.</para>
        /// </summary>
        public bool ConfigureCoupling(byte shadowId, byte primaryId, bool reversed)
        {
            lock (_io)
            {
                if (!Ready(out _)) return false;

                DynamixelSDK.Write1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, shadowId,
                    ControlTable.TorqueEnable, 0);
                if (!LastTransactionOk(out string why))
                    return Fail($"Motor ID {shadowId}: no se pudo apagar el par ({why}).");

                DynamixelSDK.Write1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, shadowId,
                    ControlTable.DriveMode, reversed ? (byte)1 : (byte)0);
                if (!LastTransactionOk(out string why2))
                    return Fail($"Motor ID {shadowId}: no se pudo escribir Drive Mode ({why2}).");

                DynamixelSDK.Write1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, shadowId,
                    ControlTable.SecondaryId, primaryId);
                if (!LastTransactionOk(out string why3))
                    return Fail($"Motor ID {shadowId}: no se pudo escribir Secondary ID ({why3}).");

                return true;
            }
        }

        // ── State ────────────────────────────────────────────────────────────

        /// <summary>
        /// One sync read of the block at 126: current, velocity and position of
        /// every joint in a single transaction. Stores the result in
        /// <see cref="LatestState"/> and returns it.
        ///
        /// <para>On failure the previous snapshot is kept and returned with
        /// <see cref="JointState.Valid"/> false, rather than handing back zeros
        /// that a controller would read as "the arm is at the origin".</para>
        /// </summary>
        public JointState ReadState()
        {
            lock (_io)
            {
                if (!Ready(out _)) return _latest.Clone();

                int group = DynamixelSDK.GroupSyncRead(_portNum, DynamixelSDK.PROTOCOL,
                    ControlTable.StateBlockStart, ControlTable.StateBlockLength);

                foreach (byte id in JointMotorIds)
                {
                    if (!DynamixelSDK.GroupSyncReadAddParam(group, id))
                    {
                        DynamixelSDK.GroupSyncReadClearParam(group);
                        Fail($"Motor ID {id}: no se pudo agregar al sync read.");
                        return Invalidate();
                    }
                }

                DynamixelSDK.GroupSyncReadTxRxPacket(group);
                bool commOk = LastTransactionOk(out string why);

                int n = JointMotorIds.Length;
                var pos = new double[n];
                var vel = new double[n];
                var cur = new double[n];
                bool complete = commOk;

                for (int i = 0; i < n && complete; i++)
                {
                    byte id = JointMotorIds[i];

                    if (!DynamixelSDK.GroupSyncReadIsAvailable(group, id,
                            ControlTable.PresentPosition, 4) ||
                        !DynamixelSDK.GroupSyncReadIsAvailable(group, id,
                            ControlTable.PresentVelocity, 4) ||
                        !DynamixelSDK.GroupSyncReadIsAvailable(group, id,
                            ControlTable.PresentCurrent, 2))
                    {
                        complete = false;
                        why = $"respuesta incompleta del motor ID {id}";
                        break;
                    }

                    uint rawPos = DynamixelSDK.GroupSyncReadGetData(group, id,
                        ControlTable.PresentPosition, 4);
                    uint rawVel = DynamixelSDK.GroupSyncReadGetData(group, id,
                        ControlTable.PresentVelocity, 4);
                    uint rawCur = DynamixelSDK.GroupSyncReadGetData(group, id,
                        ControlTable.PresentCurrent, 2);

                    pos[i] = rawPos * ControlTable.DegPerTick;
                    vel[i] = ControlTable.ToSigned32(rawVel) * ControlTable.DegPerSecPerVelUnit;
                    cur[i] = ControlTable.ToSigned16(rawCur) * ControlTable.MilliampsPerCurrentUnit;
                }

                DynamixelSDK.GroupSyncReadClearParam(group);

                if (!complete)
                {
                    Fail($"Sync read fallido: {why}.");
                    return Invalidate();
                }

                _latest = new JointState
                {
                    Position  = pos,
                    Velocity  = vel,
                    Current   = cur,
                    Timestamp = DateTime.UtcNow,
                    Valid     = true,
                };
                return _latest.Clone();
            }
        }

        // ── Goals ────────────────────────────────────────────────────────────

        /// <summary>
        /// Writes one goal per joint in a single sync write. Which register is
        /// used, and what the values mean, follows <see cref="Mode"/>:
        ///
        ///   Position / ExtendedPosition / CurrentPosition — Dynamixel degrees
        ///   Velocity — deg/s
        ///   Current  — mA
        ///   Pwm      — percent
        ///
        /// Only the primary motor of each joint is addressed; the shadows follow
        /// through their Secondary ID.
        /// </summary>
        public bool WriteGoals(double[] values)
        {
            if (values.Length != JointMotorIds.Length)
                return Fail($"WriteGoals espera {JointMotorIds.Length} valores, recibió {values.Length}.");

            (ushort address, ushort length) = Mode switch
            {
                OperatingMode.Position or
                OperatingMode.ExtendedPosition or
                OperatingMode.CurrentPosition => (ControlTable.GoalPosition, (ushort)4),
                OperatingMode.Velocity        => (ControlTable.GoalVelocity, (ushort)4),
                OperatingMode.Current         => (ControlTable.GoalCurrent,  (ushort)2),
                OperatingMode.Pwm             => (ControlTable.GoalPwm,      (ushort)2),
                _ => (ControlTable.GoalPosition, (ushort)4),
            };

            lock (_io)
            {
                if (!Ready(out _)) return false;

                int group = DynamixelSDK.GroupSyncWrite(_portNum, DynamixelSDK.PROTOCOL,
                    address, length);

                for (int i = 0; i < values.Length; i++)
                {
                    uint raw = Encode(values[i], Mode);
                    if (!DynamixelSDK.GroupSyncWriteAddParam(group, JointMotorIds[i], raw, length))
                    {
                        DynamixelSDK.GroupSyncWriteClearParam(group);
                        return Fail($"Motor ID {JointMotorIds[i]}: no se pudo agregar al sync write.");
                    }
                }

                DynamixelSDK.GroupSyncWriteTxPacket(group);
                bool ok = LastTransactionOk(out string why);
                DynamixelSDK.GroupSyncWriteClearParam(group);

                return ok || Fail($"Sync write fallido: {why}.");
            }
        }

        private static uint Encode(double value, OperatingMode mode) => mode switch
        {
            OperatingMode.Position or OperatingMode.CurrentPosition =>
                (uint)Math.Clamp((int)Math.Round(value * ControlTable.TickPerDeg), 0, 4095),

            // Multi-turn: the range is far wider and negative ticks are legal.
            OperatingMode.ExtendedPosition =>
                ControlTable.FromSigned32((int)Math.Round(value * ControlTable.TickPerDeg)),

            OperatingMode.Velocity =>
                ControlTable.FromSigned32((int)Math.Round(value / ControlTable.DegPerSecPerVelUnit)),

            OperatingMode.Current =>
                ControlTable.FromSigned16((int)Math.Round(value / ControlTable.MilliampsPerCurrentUnit)),

            OperatingMode.Pwm =>
                ControlTable.FromSigned16((int)Math.Round(value / ControlTable.PercentPerPwmUnit)),

            _ => 0u,
        };

        // ── Emergency stop ───────────────────────────────────────────────────

        /// <summary>
        /// Freezes the arm where it is: reads the present position and writes it
        /// straight back as the goal, leaving torque ON. The arm holds its pose
        /// instead of dropping, which is what a stop should do on a machine with
        /// no brakes.
        ///
        /// <para><b>Fuera de los modos de posición vuelve a modo posición.</b>
        /// Corriente, velocidad y PWM ignoran <c>Goal Position</c>, así que ahí no
        /// hay pose que sujetar: reescribirla no detendría nada. La secuencia es
        /// par off → modo posición → <see cref="EnableTorqueHolding"/>, y sí
        /// cuesta una caída breve mientras el par está apagado — el cambio de modo
        /// vive en EEPROM y no admite otra cosa.</para>
        ///
        /// <para>Esa caída es el precio de que el paro <b>sostenga de verdad</b>.
        /// La alternativa que se probó primero —escribir la corriente de gravedad
        /// y dejar el brazo flotando— <b>no es un paro</b>: es lazo abierto, sólo
        /// equilibra en la pose exacta donde se calculó, y si se mueve el brazo
        /// con la mano esa corriente constante deja de corresponder y puede
        /// empujarlo. Un paro no puede depender de que haya un lazo corriendo.</para>
        ///
        /// <para>Se sujeta donde el brazo <b>queda</b> tras la caída, no donde
        /// estaba: <see cref="EnableTorqueHolding"/> lee la pose fresca. Mandarlo
        /// de vuelta a la de antes sería comandar un movimiento, que es lo
        /// contrario de detenerse.</para>
        /// </summary>
        public bool EmergencyStop()
        {
            // Same reason EnableTorqueHolding takes it: the lock is reentrant, so
            // ReadState and WriteGoals taking it again is fine, and holding it
            // across both is what makes read-then-write atomic. Without it a tick
            // could slip between the two and the arm would freeze at a position
            // it has already left.
            lock (_io)
            {
                if (Mode is OperatingMode.Position or
                            OperatingMode.ExtendedPosition or
                            OperatingMode.CurrentPosition)
                {
                    JointState s = ReadState();
                    if (!s.Valid)
                        return Fail("Paro de emergencia: no se pudo leer la posición actual.");

                    return WriteGoals(s.Position);
                }

                // Antes de soltar el par, quitar el mando que el brazo esté
                // siguiendo. En corriente eso es corriente cero; si no, el motor
                // conserva la última meta escrita durante todo el cambio de modo.
                if (Mode == OperatingMode.Current && !WriteGoals(new double[JointCount]))
                    return Fail("Paro de emergencia: no se pudo poner la corriente a cero. " +
                                Environment.NewLine + LastError);

                if (!SetOperatingMode(OperatingMode.Position))
                    return Fail("Paro de emergencia: no se pudo volver a modo posición. " +
                                Environment.NewLine + LastError);

                // SetOperatingMode deja el par apagado a propósito (§4.1), así
                // que el brazo está cayendo desde aquí hasta la línea siguiente.
                if (!EnableTorqueHolding())
                    return Fail("Paro de emergencia: se volvió a modo posición pero " +
                                "no se pudo sujetar la pose. " + Environment.NewLine + LastError);

                return true;
            }
        }

        // ── Gripper ──────────────────────────────────────────────────────────
        // Kept apart from the joints on purpose: it is an end effector and will
        // grow its own interface. It is also detachable, so both calls below
        // check HasGripper first and say so, instead of spending a transaction
        // to come back with a timeout that reads like a bus fault.

        /// <summary>
        /// Gripper position, in Dynamixel degrees. NaN if it cannot be read,
        /// including when no gripper was found at connect.
        /// </summary>
        public double GetGripperPosition()
        {
            lock (_io)
            {
                if (!Ready(out _)) return double.NaN;
                if (!HasGripper)
                {
                    Fail($"El gripper (ID {GripperId}) no estaba conectado al abrir el puerto.");
                    return double.NaN;
                }

                uint raw = DynamixelSDK.Read4ByteTxRx(_portNum, DynamixelSDK.PROTOCOL,
                    GripperId, ControlTable.PresentPosition);
                if (!LastTransactionOk(out string why))
                {
                    Fail($"Gripper: no se pudo leer la posición ({why}).");
                    return double.NaN;
                }
                return raw * ControlTable.DegPerTick;
            }
        }

        /// <summary>Commands the gripper, in Dynamixel degrees.</summary>
        public bool SetGripperPosition(double degrees)
        {
            lock (_io)
            {
                if (!Ready(out _)) return false;
                if (!HasGripper)
                    return Fail($"El gripper (ID {GripperId}) no estaba conectado al abrir el puerto.");

                uint ticks = (uint)Math.Clamp((int)Math.Round(degrees * ControlTable.TickPerDeg), 0, 4095);
                DynamixelSDK.Write4ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, GripperId,
                    ControlTable.GoalPosition, ticks);
                if (!LastTransactionOk(out string why))
                    return Fail($"Gripper: no se pudo escribir la posición ({why}).");
                return true;
            }
        }

        // ── IDisposable ──────────────────────────────────────────────────────

        public void Dispose() => Disconnect();

        // ── Helpers ──────────────────────────────────────────────────────────

        /// <summary>
        /// Checks the outcome of the transaction that just ran. Both halves
        /// matter: the comm result says whether the packet made it, the packet
        /// error says whether the motor rejected it.
        /// </summary>
        private bool LastTransactionOk(out string reason)
        {
            int comm = DynamixelSDK.GetLastTxRxResult(_portNum, DynamixelSDK.PROTOCOL);
            if (comm != DynamixelSDK.COMM_SUCCESS)
            {
                reason = $"sin respuesta (comm={comm})";
                return false;
            }

            byte err = DynamixelSDK.GetLastRxPacketError(_portNum, DynamixelSDK.PROTOCOL);
            if (err != 0)
            {
                reason = $"el motor rechazó el paquete (err=0x{err:X2})";
                return false;
            }

            reason = string.Empty;
            return true;
        }

        /// <summary>
        /// Writes one 4-byte RAM register on every joint motor, checking each
        /// transaction. Used by the profile setters, which apply the same value
        /// to all joints.
        /// </summary>
        private bool WriteAllJoints4(ushort address, uint value, string what)
        {
            lock (_io)
            {
                if (!Ready(out _)) return false;
                foreach (byte id in JointMotorIds)
                {
                    DynamixelSDK.Write4ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id, address, value);
                    if (!LastTransactionOk(out string why))
                        return Fail($"Motor ID {id}: no se pudo escribir {what} ({why}).");
                }
                return true;
            }
        }

        private bool Ready(out string reason)
        {
            if (_portNum < 0 || !_connected)
            {
                reason = "el dispositivo no está conectado";
                LastError = reason;
                return false;
            }
            reason = string.Empty;
            return true;
        }

        private JointState Invalidate()
        {
            _latest = new JointState
            {
                Position  = _latest.Position,
                Velocity  = _latest.Velocity,
                Current   = _latest.Current,
                Timestamp = DateTime.UtcNow,
                Valid     = false,
            };
            return _latest.Clone();
        }

        private bool Fail(string reason)
        {
            LastError = reason;
            return false;
        }
    }
}
