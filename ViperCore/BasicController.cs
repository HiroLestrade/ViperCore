namespace ViperCore
{
    /// <summary>
    /// Passes the trajectory's desired position directly as the output command.
    /// Dynamixel motors have built-in position control, so no external PID is needed.
    /// Equivalent role to PIDController in GeomagicCore.
    /// </summary>
    public sealed class BasicController : IController
    {
        public void Reset() { }

        public void Compute(ControlInput input, double[] output)
        {
            Array.Copy(input.Qd, output, output.Length);
        }
    }
}
