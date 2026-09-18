namespace ViperCore
{
    /// <summary>
    /// Reference generator. Produces position, velocity and acceleration so a
    /// controller can feed the last two forward instead of relying on error
    /// alone.
    /// </summary>
    public interface ITrajectory
    {
        /// <summary>Called once at the start of a motion.</summary>
        void Init(double[] q0, double[] qf, double tf);

        /// <summary>
        /// Reference at time <paramref name="t"/> (seconds since Init).
        /// <paramref name="qpd"/> and <paramref name="qppd"/> may be null when
        /// the caller does not need them.
        /// </summary>
        void Evaluate(double t, double[] qd, double[]? qpd, double[]? qppd);
    }
}
