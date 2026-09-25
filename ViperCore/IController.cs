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

        /// <summary>
        /// The ceiling this controller's output must not exceed, per joint, in the
        /// units of <see cref="RequiredMode"/> — or <c>null</c> for no declared
        /// ceiling, which is the default and what every position controller wants.
        ///
        /// <para>It is the controller that declares it because the controller is
        /// what knows how much it can legitimately ask for. A gravity compensator
        /// knows the largest current the model can produce; a computed-torque law
        /// does not have the same envelope.</para>
        ///
        /// <para><see cref="JointController"/> uses it twice, and the two are not
        /// redundant: it writes it into the motors' <c>Current Limit</c> when the
        /// controller is installed, so the firmware enforces it even if this
        /// software stops writing, and it clamps the output every tick, so a
        /// command past the ceiling is <b>reported</b> rather than silently
        /// saturated by the firmware. Without the second one there is no way to
        /// tell "the model asked for 8 A and got clipped" from "the model asked for
        /// exactly the limit".</para>
        /// </summary>
        double[]? OutputLimit => null;

        /// <summary>Called once at the start of each motion.</summary>
        void Reset();

        /// <summary>
        /// Computes one command per joint, in the units of
        /// <see cref="RequiredMode"/>.
        /// </summary>
        void Compute(ControlInput input, double[] output);
    }
}
