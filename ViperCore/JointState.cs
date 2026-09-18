namespace ViperCore
{
    /// <summary>
    /// One snapshot of the arm, as measured. All three quantities come from the
    /// motors themselves in a single sync read — the velocity is **not**
    /// differentiated from the position.
    ///
    /// Units:
    ///   Position  Dynamixel degrees, 0–360, 180 = centre
    ///   Velocity  degrees per second, signed
    ///   Current   milliamps, signed
    /// </summary>
    public sealed class JointState
    {
        public double[] Position { get; init; } = [];
        public double[] Velocity { get; init; } = [];
        public double[] Current  { get; init; } = [];

        /// <summary>When the sync read that produced this snapshot returned.</summary>
        public DateTime Timestamp { get; init; }

        /// <summary>
        /// False when the sync read failed or came back incomplete. The values
        /// are then the previous snapshot's, not fresh ones.
        /// </summary>
        public bool Valid { get; init; }

        public int Count => Position.Length;

        public static JointState Empty(int n) => new()
        {
            Position  = new double[n],
            Velocity  = new double[n],
            Current   = new double[n],
            Timestamp = DateTime.UtcNow,
            Valid     = false,
        };

        public JointState Clone() => new()
        {
            Position  = (double[])Position.Clone(),
            Velocity  = (double[])Velocity.Clone(),
            Current   = (double[])Current.Clone(),
            Timestamp = Timestamp,
            Valid     = Valid,
        };
    }

    /// <summary>
    /// What <see cref="ViperDevice.VerifyCoupling"/> found on one of the
    /// secondary motors of a dual-motor joint.
    /// </summary>
    public sealed record CouplingStatus(
        byte SecondaryMotorId,
        byte PrimaryMotorId,
        byte SecondaryIdRegister,
        byte DriveModeRegister)
    {
        /// <summary>
        /// True when this motor is set to shadow its primary, so a write to the
        /// primary drives both.
        /// </summary>
        public bool IsShadowing => SecondaryIdRegister == PrimaryMotorId;

        /// <summary>Bit 0 of Drive Mode: the motor turns the other way.</summary>
        public bool IsReversed => (DriveModeRegister & 0x01) != 0;

        public override string ToString() =>
            $"ID {SecondaryMotorId}: Secondary ID = {SecondaryIdRegister} " +
            $"(esperado {PrimaryMotorId}), Drive Mode = 0x{DriveModeRegister:X2} " +
            $"({(IsReversed ? "invertido" : "normal")})";
    }
}
