namespace ViperCore
{
    /// <summary>
    /// Operating mode of a Dynamixel XM/XH motor (control table address 11).
    ///
    /// The mode decides which goal register the motor obeys, and therefore what
    /// units an <see cref="IController"/> must produce:
    ///
    ///   Current            -> Goal Current  (mA)
    ///   Velocity           -> Goal Velocity (deg/s)
    ///   Position           -> Goal Position (Dynamixel degrees)
    ///   ExtendedPosition   -> Goal Position, multi-turn range
    ///   CurrentPosition    -> Goal Position, with the current capped by Goal Current
    ///   Pwm                -> Goal PWM (percent of the PWM limit)
    ///
    /// Operating Mode lives in EEPROM, so it can only be written while torque is
    /// DISABLED. There is no way to change it on a live motor: the sequence is
    /// always stop -> torque off -> set mode -> configure limits -> torque on.
    /// </summary>
    public enum OperatingMode
    {
        Current          = 0,
        Velocity         = 1,
        Position         = 3,
        ExtendedPosition = 4,
        CurrentPosition  = 5,
        Pwm              = 16,
    }

    /// <summary>
    /// Control table of the XM430-W350 / XM540-W270 under Protocol 2.0, and the
    /// scale factors of the registers this project touches.
    ///
    /// Addresses 0–63 are EEPROM: writable only with torque disabled, and with a
    /// finite number of write cycles (~100k). Addresses 64 and up are RAM and
    /// can be written freely while the motor runs.
    /// </summary>
    internal static class ControlTable
    {
        // ── EEPROM (torque must be off to write) ─────────────────────────────
        public const ushort DriveMode        = 10;   // 1 B — bit 0: 0 normal, 1 reverse
        public const ushort OperatingMode    = 11;   // 1 B
        public const ushort SecondaryId      = 12;   // 1 B — 255 = disabled
        public const ushort CurrentLimit     = 38;   // 2 B

        // ── RAM ──────────────────────────────────────────────────────────────
        public const ushort TorqueEnable     = 64;   // 1 B
        public const ushort HardwareError    = 70;   // 1 B
        public const ushort GoalPwm          = 100;  // 2 B  signed
        public const ushort GoalCurrent      = 102;  // 2 B  signed
        public const ushort GoalVelocity     = 104;  // 4 B  signed
        public const ushort ProfileAccel     = 108;  // 4 B
        public const ushort ProfileVelocity  = 112;  // 4 B
        public const ushort GoalPosition     = 116;  // 4 B
        public const ushort PresentCurrent   = 126;  // 2 B  signed
        public const ushort PresentVelocity  = 128;  // 4 B  signed
        public const ushort PresentPosition  = 132;  // 4 B

        /// <summary>
        /// Present Current, Velocity and Position sit contiguous at 126..135, so
        /// a single sync read of ten bytes brings back all three from every
        /// motor in one transaction.
        /// </summary>
        public const ushort StateBlockStart  = PresentCurrent;
        public const ushort StateBlockLength = 10;

        // ── Scale factors ────────────────────────────────────────────────────

        /// <summary>0.088° per tick — 4096 ticks over 360°.</summary>
        public const double DegPerTick = 360.0 / 4096.0;
        public const double TickPerDeg = 4096.0 / 360.0;

        /// <summary>
        /// 0.229 rev/min per unit, i.e. 0.229 · 360/60 = 1.374 deg/s per unit.
        /// </summary>
        public const double DegPerSecPerVelUnit = 0.229 * 360.0 / 60.0;

        /// <summary>2.69 mA per unit, for both XM430-W350 and XM540-W270.</summary>
        public const double MilliampsPerCurrentUnit = 2.69;

        /// <summary>Goal PWM and Present PWM are 0.113 % per unit.</summary>
        public const double PercentPerPwmUnit = 0.113;

        // ── Helpers for the signed registers ─────────────────────────────────
        // Sync reads hand back raw unsigned words; velocity and current are
        // two's-complement and have to be reinterpreted.

        public static int ToSigned32(uint raw) => unchecked((int)raw);

        public static int ToSigned16(uint raw) => unchecked((short)(ushort)raw);

        public static uint FromSigned32(int value) => unchecked((uint)value);

        public static uint FromSigned16(int value) => unchecked((ushort)(short)value);
    }

    /// <summary>
    /// Bits of the Hardware Error Status register (address 70). Any bit set means
    /// the motor has latched a fault; on overload or overheating it disables its
    /// own torque, which on an arm without brakes means the arm drops. Worth
    /// surfacing rather than letting the fall look inexplicable.
    /// </summary>
    [Flags]
    public enum HardwareError : byte
    {
        None            = 0,
        InputVoltage    = 1 << 0,
        Overheating     = 1 << 2,
        MotorEncoder    = 1 << 3,
        ElectricalShock = 1 << 4,
        Overload        = 1 << 5,
    }
}
