namespace ViperCore
{
    /// <summary>
    /// A joint controller. Beyond computing a command, it declares which
    /// operating mode the motors must be in for that command to mean anything —
    /// a position controller and a current controller write different registers.
    ///
    /// <see cref="JointController"/> applies <see cref="RequiredMode"/> when the
    /// controller is installed, so the mode follows the controller instead of
    /// being a separate setting somebody has to remember to match.
    ///
    /// Changing mode needs torque off (it lives in EEPROM), so the switch cannot
    /// happen while the arm is running: install the controller first, then start.
    /// </summary>
    public interface IController
    {
        /// <summary>Operating mode this controller's output is expressed in.</summary>
        OperatingMode RequiredMode { get; }

        /// <summary>Called once at the start of each motion.</summary>
        void Reset();

        /// <summary>
        /// Computes one command per joint, in the units of
        /// <see cref="RequiredMode"/>.
        /// </summary>
        void Compute(ControlInput input, double[] output);
    }
}
