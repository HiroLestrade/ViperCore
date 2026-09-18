namespace ViperCore
{
    /// <summary>
    /// State handed to <see cref="IController.Compute"/> each tick.
    ///
    /// <para><b>Q and Qp are measured, not derived.</b> The Dynamixel motors
    /// report Present Velocity directly (control table address 128), so the
    /// velocity here comes off the bus rather than out of a differentiator. Only
    /// <see cref="Qpp"/> is computed, and it differentiates a measured velocity
    /// once instead of a measured position twice — a much cleaner signal.</para>
    ///
    /// <para>Units: positions in Dynamixel degrees (0–360, 180 = centre),
    /// velocities in deg/s, accelerations in deg/s², currents in mA.</para>
    /// </summary>
    public sealed class ControlInput
    {
        // ── Measured ─────────────────────────────────────────────────────────

        /// <summary>Present position (deg).</summary>
        public double[] Q { get; set; } = [];

        /// <summary>Present velocity as reported by the motors (deg/s).</summary>
        public double[] Qp { get; set; } = [];

        /// <summary>Acceleration, differentiated from <see cref="Qp"/> (deg/s²).</summary>
        public double[] Qpp { get; set; } = [];

        /// <summary>Present current (mA). Proportional to the torque delivered.</summary>
        public double[] Current { get; set; } = [];

        // ── Desired, from the trajectory ─────────────────────────────────────

        /// <summary>Desired position (deg).</summary>
        public double[] Qd { get; set; } = [];

        /// <summary>Desired velocity (deg/s), for feedforward.</summary>
        public double[] Qpd { get; set; } = [];

        /// <summary>Desired acceleration (deg/s²), for feedforward.</summary>
        public double[] Qppd { get; set; } = [];

        // ── Timing ───────────────────────────────────────────────────────────

        /// <summary>Seconds since the motion started.</summary>
        public double T { get; set; }

        /// <summary>
        /// Elapsed time since the previous tick, in seconds. This is the
        /// **measured** interval, not the nominal period: the tick runs on a
        /// timer over a serial bus and its real spacing wanders.
        /// </summary>
        public double Dt { get; set; }
    }
}
