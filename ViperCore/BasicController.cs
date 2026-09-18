namespace ViperCore
{
    /// <summary>
    /// Forwards the trajectory's desired position straight to the motors and
    /// lets their internal position servo close the loop. Runs in
    /// <see cref="OperatingMode.Position"/>.
    ///
    /// This is the baseline: no external feedback, so nothing here can
    /// destabilise. A controller that closes its own loop — a joint PID, say —
    /// would declare <see cref="OperatingMode.Current"/> or
    /// <see cref="OperatingMode.Pwm"/> instead, and then <see cref="ControlInput"/>'s
    /// measured position, velocity and current become the signals it works on.
    /// </summary>
    public sealed class BasicController : IController
    {
        public OperatingMode RequiredMode => OperatingMode.Position;

        public void Reset() { }

        public void Compute(ControlInput input, double[] output)
        {
            Array.Copy(input.Qd, output, output.Length);
        }
    }
}
