namespace ViperCore
{
    /// <summary>
    /// The arm's mechanical joint limits, read off the motors' EEPROM
    /// (<c>Min/Max Position Limit</c>) and kept here so there is one copy.
    ///
    /// <para>These are <b>not</b> in the article's model, and they are the last
    /// geometric guard the kinematics was missing: <see cref="ViperKinematics"/>
    /// could check that a target was within reach and still hand back a pose the
    /// linkage refuses, because three of the six joints stop well short of a full
    /// turn.</para>
    ///
    /// <para>The stored values are in <b>Dynamixel degrees</b>, exactly as the arm
    /// reports them, because that is the form that can be checked against the
    /// hardware without arithmetic in between. Real degrees and model radians are
    /// derived from them once, at class load.</para>
    ///
    /// <para><b>What they are not.</b> These are the limits the <i>motor</i>
    /// enforces. A pose inside all six can still be a self-collision — the
    /// forearm meeting the shoulder, the gripper meeting the base — because a
    /// per-joint interval cannot express a limit that depends on another joint.
    /// Nothing here checks that.</para>
    /// </summary>
    public static class ViperJointLimits
    {
        /// <summary>
        /// One encoder count, in degrees. The XM/XH series resolves a turn into
        /// 4096 counts, which is why every limit below ends in a multiple of this
        /// and not in a round number.
        /// </summary>
        public const double CountDeg = 360.0 / 4096.0;   // 0.087890625

        /// <summary>
        /// Lower limits, Dynamixel degrees. Index 0 is q1.
        ///
        /// <para><b>q2's is 65.04, not the factory 73.92.</b> It was widened on the
        /// arm deliberately, because the rest pose needs the shoulder further back
        /// than the stock limit allows. That is why this table is read off the
        /// motors rather than copied from the ViperX-300 documentation: the arm is
        /// the authority on its own limits, and this one no longer matches the
        /// published figure.</para>
        /// </summary>
        private static readonly double[] MinDxl =
            [0.00, 65.04, 78.93, 0.00, 72.95, 0.00];

        /// <summary>
        /// Upper limits, Dynamixel degrees.
        ///
        /// <para>The waist and the wrist roll reach 359.91° rather than 360°
        /// because 359.91 <i>is</i> the last count: the range is the full turn
        /// minus one step, and 0 and 359.91 are distinct commands with no
        /// wraparound between them.</para>
        /// </summary>
        private static readonly double[] MaxDxl =
            [359.91, 251.90, 271.93, 350.91, 307.97, 359.91];

        private static readonly double[] MinRadArr = new double[ViperKinematics.JointCount];
        private static readonly double[] MaxRadArr = new double[ViperKinematics.JointCount];

        static ViperJointLimits()
        {
            for (int i = 0; i < ViperKinematics.JointCount; i++)
            {
                MinRadArr[i] = ViperAngles.ToModelRad(MinDxl[i]);
                MaxRadArr[i] = ViperAngles.ToModelRad(MaxDxl[i]);
            }
        }

        /// <summary>Lower limit of a joint, Dynamixel degrees.</summary>
        public static double MinDynamixelDeg(int joint) => MinDxl[joint];

        /// <summary>Upper limit of a joint, Dynamixel degrees.</summary>
        public static double MaxDynamixelDeg(int joint) => MaxDxl[joint];

        /// <summary>Lower limit of a joint, real degrees — zero at centre.</summary>
        public static double MinRealDeg(int joint) => MinDxl[joint] - ViperAngles.CentreDeg;

        /// <summary>Upper limit of a joint, real degrees.</summary>
        public static double MaxRealDeg(int joint) => MaxDxl[joint] - ViperAngles.CentreDeg;

        /// <summary>Lower limit of a joint, model radians.</summary>
        public static double MinRad(int joint) => MinRadArr[joint];

        /// <summary>Upper limit of a joint, model radians.</summary>
        public static double MaxRad(int joint) => MaxRadArr[joint];

        /// <summary>
        /// Tolerance on every test below: one encoder count, in the unit being
        /// tested.
        ///
        /// <para>It is there because the limits are quantised to counts and the
        /// angles tested against them are not. A solution that lands 0.05° past a
        /// limit is <i>at</i> that limit as far as the motor is concerned, since
        /// the command rounds to the same count either way; rejecting it would
        /// refuse a pose the arm holds perfectly well.</para>
        /// </summary>
        public const double SlackDeg = CountDeg;

        private const double SlackRad = CountDeg * Math.PI / 180.0;

        /// <summary>Whether a joint angle in model radians is within its limits.</summary>
        public static bool Contains(int joint, double qRad) =>
            qRad >= MinRadArr[joint] - SlackRad && qRad <= MaxRadArr[joint] + SlackRad;

        /// <summary>Whether a joint angle in real degrees is within its limits.</summary>
        public static bool ContainsRealDeg(int joint, double realDeg) =>
            realDeg >= MinRealDeg(joint) - SlackDeg && realDeg <= MaxRealDeg(joint) + SlackDeg;

        /// <summary>
        /// The equivalent of <paramref name="qRad"/> that this joint can actually
        /// be commanded to, if there is one.
        ///
        /// <para>Two angles a full turn apart are the same configuration, but only
        /// one of them is a command a Dynamixel in position mode accepts, and
        /// which one depends on where that joint's limits sit. On the waist and
        /// the wrist roll the range is a whole turn less a count, so +180° is
        /// <i>not</i> expressible while −180° is, even though they are the same
        /// place. Wrapping to a symmetric ±180 and hoping is what this
        /// replaces.</para>
        ///
        /// <para>Returns the input unchanged when neither branch fits, so the
        /// caller's limit test reports the real number rather than a shifted
        /// one.</para>
        /// </summary>
        public static double Fold(int joint, double qRad)
        {
            const double Turn = 2.0 * Math.PI;

            if (qRad > MaxRadArr[joint] + SlackRad &&
                qRad - Turn >= MinRadArr[joint] - SlackRad)
                return qRad - Turn;

            if (qRad < MinRadArr[joint] - SlackRad &&
                qRad + Turn <= MaxRadArr[joint] + SlackRad)
                return qRad + Turn;

            return qRad;
        }

        /// <summary>
        /// Why an angle is outside this joint's limits, for a user. Speaks real
        /// degrees, because that is the only convention the panel shows.
        /// </summary>
        public static string Describe(int joint, double qRad) =>
            DescribeRealDeg(joint, qRad * 180.0 / Math.PI);

        /// <summary>The same message for a value already in real degrees.</summary>
        public static string DescribeRealDeg(int joint, double realDeg)
        {
            double min = MinRealDeg(joint), max = MaxRealDeg(joint);

            string side = realDeg > max
                ? $"pasa el tope superior por {realDeg - max:F2}°"
                : $"pasa el tope inferior por {min - realDeg:F2}°";

            return $"q{joint + 1} = {realDeg:F2}° {side}. " +
                   $"El rango mecánico de q{joint + 1} es de {min:F2}° a {max:F2}°.";
        }

        /// <summary>
        /// Checks a whole pose in real degrees, the panel's convention, and says
        /// which joint failed first.
        /// </summary>
        public static bool TryCheckRealDeg(double[] realDeg, out string reason)
        {
            reason = string.Empty;

            for (int i = 0; i < realDeg.Length && i < ViperKinematics.JointCount; i++)
            {
                if (ContainsRealDeg(i, realDeg[i])) continue;

                reason = DescribeRealDeg(i, realDeg[i]);
                return false;
            }

            return true;
        }
    }
}
