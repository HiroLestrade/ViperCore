namespace ViperCore
{
    /// <summary>
    /// Manages the connection to a Viper X-300S arm via Dynamixel Protocol 2.0.
    /// Mirrors the GeomagicDevice pattern: Connect / GetJointAngles / SetJointAngles.
    /// </summary>
    public sealed class ViperDevice : IDisposable
    {
        // ── Viper X-300S motor map ────────────────────────────────────────────────
        //
        //  Joint  │ ID(s)  │ Model
        //  ───────┼────────┼────────────
        //  q1     │  1     │ XM540-W70
        //  q2     │  2, 3  │ XM540-W70  (dual — 3 mirrors 2)
        //  q3     │  4, 5  │ XM540-W70  (dual — 5 mirrors 4)
        //  q4     │  6     │ XM540-W70
        //  q5     │  7     │ XM540-W70
        //  q6     │  8     │ XM430-W350
        //  gripper│  9     │ XM430-W350
        //
        // All IDs — used for connection verification (ping).
        private static readonly byte[] AllMotorIds = [1, 2, 3, 4, 5, 6, 7, 8, 9];

        // Primary motor per joint — used for reading/writing angles.
        // Secondary motors (3, 5) mirror their primaries automatically.
        private static readonly byte[] JointMotorIds = [1, 2, 4, 6, 7, 8, 9];

        // ── XM430-W350 control table addresses ───────────────────────────────────

        private const ushort ADDR_TORQUE_ENABLE    = 64;
        private const ushort ADDR_GOAL_POSITION    = 116;
        private const ushort ADDR_PRESENT_POSITION = 132;

        // ── XM430/XM540 position ↔ Dynamixel-degree conversion ───────────────────
        //
        // Dynamixel degrees: 0° = one extreme, 180° = center (neutral), 360° = other extreme.
        // This matches what Dynamixel Wizard displays — no center-offset abstraction needed.

        private const double TICKS_TO_DEG = 360.0 / 4096.0;
        private const double DEG_TO_TICKS = 4096.0 / 360.0;

        // ── State ────────────────────────────────────────────────────────────────

        private int  _portNum = -1;
        private bool _connected;

        public bool    IsConnected => _connected;
        public string? LastError   { get; private set; }

        // ── Connection ───────────────────────────────────────────────────────────

        public bool Connect(string portName, int baudRate = 1_000_000)
        {
            Disconnect();

            _portNum = DynamixelSDK.PortHandler(portName);
            DynamixelSDK.PacketHandler();   // initializes internal state; no return value used

            if (!DynamixelSDK.OpenPort(_portNum))
                return Fail($"No se pudo abrir el puerto {portName}.");

            if (!DynamixelSDK.SetBaudRate(_portNum, baudRate))
                return Fail($"No se pudo configurar baud rate {baudRate}.");

            foreach (byte id in AllMotorIds)
            {
                ushort model = 0;
                byte   error = 0;

                // ping() returns the model number, not the comm result.
                // Use GetLastTxRxResult to check whether communication succeeded.
                DynamixelSDK.Ping(_portNum, DynamixelSDK.PROTOCOL, id, ref model, ref error);

                int comm = DynamixelSDK.GetLastTxRxResult(_portNum, DynamixelSDK.PROTOCOL);
                if (comm != DynamixelSDK.COMM_SUCCESS)
                    return Fail($"Motor ID {id}: sin respuesta (comm={comm}).");
                if (error != 0)
                    return Fail($"Motor ID {id}: error de hardware (err=0x{error:X2}).");
            }

            _connected = true;
            LastError  = null;
            return true;
        }

        public void Disconnect()
        {
            if (_connected)
                EnableTorque(false);

            if (_portNum >= 0)
            {
                DynamixelSDK.ClosePort(_portNum);
                _portNum = -1;
            }

            _connected = false;
        }

        // ── Torque ───────────────────────────────────────────────────────────────

        public void EnableTorque(bool enable)
        {
            if (_portNum < 0) return;
            byte value = enable ? (byte)1 : (byte)0;
            byte error = 0;
            foreach (byte id in AllMotorIds)
                DynamixelSDK.Write1ByteTxRx(_portNum, DynamixelSDK.PROTOCOL, id,
                    ADDR_TORQUE_ENABLE, value, ref error);
        }

        // ── Joint state ──────────────────────────────────────────────────────────

        /// <summary>
        /// Reads present position from all six motors and returns joint angles in radians.
        /// </summary>
        /// <summary>
        /// Reads present position from the six arm joints (one motor per joint).
        /// Returns angles in radians. Gripper is excluded.
        /// </summary>
        public double[] GetJointAngles()
        {
            double[] q     = new double[JointMotorIds.Length];
            byte     error = 0;

            for (int i = 0; i < JointMotorIds.Length; i++)
            {
                uint raw = DynamixelSDK.Read4ByteTxRx(_portNum, DynamixelSDK.PROTOCOL,
                    JointMotorIds[i], ADDR_PRESENT_POSITION, ref error);

                q[i] = raw * TICKS_TO_DEG;
            }

            return q;
        }

        /// <summary>
        /// Sends goal positions to the six arm joints (radians).
        /// Secondary motors of dual joints (IDs 3, 5) follow automatically.
        /// </summary>
        public void SetJointAngles(double[] q)
        {
            byte error = 0;
            for (int i = 0; i < JointMotorIds.Length; i++)
            {
                uint ticks = (uint)Math.Clamp(
                    (int)Math.Round(q[i] * DEG_TO_TICKS),
                    0, 4095);

                DynamixelSDK.Write4ByteTxRx(_portNum, DynamixelSDK.PROTOCOL,
                    JointMotorIds[i], ADDR_GOAL_POSITION, ticks, ref error);
            }
        }

        // ── IDisposable ──────────────────────────────────────────────────────────

        public void Dispose() => Disconnect();

        // ── Helpers ──────────────────────────────────────────────────────────────

        private bool Fail(string reason)
        {
            LastError = reason;
            if (_portNum >= 0)
            {
                DynamixelSDK.ClosePort(_portNum);
                _portNum = -1;
            }
            _connected = false;
            return false;
        }
    }
}
