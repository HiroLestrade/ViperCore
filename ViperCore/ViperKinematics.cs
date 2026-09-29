namespace ViperCore
{
    /// <summary>
    /// Forward kinematics of the Viper X-300S, from the model identified in
    /// Momani &amp; Hosseinzadeh, "Physically feasible dynamic model identification
    /// and constrained control of robotic arms — a case study on the ViperX-300
    /// 6-DoF robotic manipulator", Mechatronics 112 (2025) 103419, Table 1.
    ///
    /// <para><b>Convention: modified (Craig) D-H.</b> The table is given as
    /// <c>α(i−1), a(i−1), d(i), θ(i)</c>, so each link transform is
    /// <c>Rot_x(α) · Trans_x(a) · Rot_z(θ) · Trans_z(d)</c>, not the classic
    /// Denavit–Hartenberg ordering. Using the classic form with these numbers
    /// produces a plausible-looking arm that is wrong.</para>
    ///
    /// <para><b>Units and angles.</b> Joint angles here are the <b>article's
    /// q, in radians</b>, zero at the manipulator's reference configuration —
    /// not Dynamixel degrees. The conversion between the two belongs above this
    /// library, with the rest of the joint-angle convention. Lengths are metres.</para>
    ///
    /// <para>This is deliberately the same D-H chain the dynamic model was
    /// derived from, so kinematics and dynamics cannot disagree about where the
    /// arm is.</para>
    /// </summary>
    public static class ViperKinematics
    {
        // ── Link lengths, metres (article §3) ────────────────────────────────

        /// <summary>Base to shoulder.</summary>
        public const double L1 = 0.12675;

        /// <summary>Shoulder to elbow.</summary>
        public const double L2 = 0.30594;

        /// <summary>Elbow to wrist, first part. L3 + L4 is the forearm, 0.300 m.</summary>
        public const double L3 = 0.19640;

        /// <summary>Elbow to wrist, second part.</summary>
        public const double L4 = 0.10362;

        /// <summary>Wrist centre to the end effector's mounting face.</summary>
        public const double L5 = 0.07;

        /// <summary>
        /// The stock gripper, from its mounting face to the fingertips — the
        /// article's L6. Kept for reference; this arm is not running it.
        /// </summary>
        public const double GripperLength = 0.13658;

        /// <summary>
        /// What is actually mounted: a <b>45 mm finger carrying the force
        /// sensor</b>, in place of the gripper. From the mounting face to the
        /// fingertip.
        ///
        /// <para>It replaced a plain 62 mm finger, and that is the one number that
        /// changed: the sensor finger points along z6 like the old one, so nothing
        /// about the geometry is different except its length.</para>
        ///
        /// <para>Change this one constant if the tool changes — it is the only thing
        /// between the joint angles and where the Cartesian readout says the tip is.
        /// It is a <b>length</b>, and only that: how much the tool weighs is a
        /// separate question that this constant has no opinion about.</para>
        /// </summary>
        public const double FingerLength = 0.045;

        /// <summary>
        /// Frame-6 origin to the tool tip, along z6 — the article's d6. Frames 4,
        /// 5 and 6 share an origin, the wrist centre, so this single length is the
        /// whole end effector, and it is what backs the tip off to the decoupling
        /// point in <see cref="Inverse"/>.
        ///
        /// <para><b>Measured, not derived</b>, and re-measured when the tool
        /// changed. It is 115 mm from the wrist centre to the tip of the 45 mm
        /// sensor finger — measured on the arm, and equal to the nominal
        /// <see cref="L5"/> + <see cref="FingerLength"/> to the millimetre. It was
        /// 130 mm with the previous 62 mm finger.</para>
        ///
        /// <para><b>The reference point is the wrist centre, not the mounting
        /// face.</b> Worth saying because the tool gets specified from the flange:
        /// a 45 mm finger is a 115 mm tool, not a 45 mm one. Putting the flange
        /// figure here would move the reported tip 70 mm — the length of
        /// <see cref="L5"/> — and the error would show up as a Cartesian offset
        /// that looks like a kinematics problem.</para>
        /// </summary>
        public const double ToolLength = 0.115;

        public const int JointCount = 6;

        // ── D-H table (Table 1) ──────────────────────────────────────────────
        //
        //  Joint │ α(i−1) │ a(i−1) │  d(i)   │   θ(i)
        //  ──────┼────────┼────────┼─────────┼──────────────
        //    1   │   0    │   0    │   L1    │ q1
        //    2   │  3π/2  │   0    │   0     │ q2 − 0.437π
        //    3   │   0    │   L2   │   0     │ q3 − 0.063π
        //    4   │  3π/2  │   0    │ L3 + L4 │ q4
        //    5   │   π/2  │   0    │   0     │ q5
        //    6   │  3π/2  │   0    │   0     │ q6
        //
        // The two θ offsets are the same biases the dynamic model subtracts
        // before evaluating M, C and G — 0.437π = 78.66°, 0.063π = 11.34°.

        private static readonly double[] Alpha =
            [0.0, 3.0 * Math.PI / 2.0, 0.0, 3.0 * Math.PI / 2.0, Math.PI / 2.0, 3.0 * Math.PI / 2.0];

        private static readonly double[] A =
            [0.0, 0.0, L2, 0.0, 0.0, 0.0];

        private static readonly double[] D =
            [L1, 0.0, 0.0, L3 + L4, 0.0, 0.0];

        private static readonly double[] ThetaOffset =
            [0.0, -0.437 * Math.PI, -0.063 * Math.PI, 0.0, 0.0, 0.0];

        // ── Forward kinematics ───────────────────────────────────────────────

        /// <summary>
        /// Tool-tip position in the base frame, metres, for six joint angles in
        /// radians. Allocates the array it returns; use
        /// <see cref="Forward(double[], double[])"/> in a control loop.
        /// </summary>
        public static double[] Forward(double[] qRad)
        {
            var p = new double[3];
            Forward(qRad, p);
            return p;
        }

        /// <summary>
        /// Tool-tip position into a caller-supplied array of three.
        ///
        /// <para><b>Allocates nothing.</b> The 4×4s the chain needs live on the
        /// stack, so a loop running at hundreds of hertz can call this inside the
        /// control period without handing the GC work — those pauses would fall
        /// inside the loop. Keep it that way: no <c>new</c> on this path.</para>
        /// </summary>
        public static void Forward(double[] qRad, double[] position)
        {
            if (position.Length < 3)
                throw new ArgumentException(
                    "La posición necesita al menos 3 elementos.", nameof(position));

            Span<double> t = stackalloc double[16];
            Chain(qRad, t, withTool: true);

            position[0] = t[3];
            position[1] = t[7];
            position[2] = t[11];
        }

        /// <summary>
        /// Full 4×4 pose of the tool frame in the base frame: rotation in the
        /// upper-left 3×3, position in the last column. Orientation is what the
        /// inverse kinematics and any force transform will need.
        ///
        /// <para>Allocates the 4×4 it returns — this is the convenient shape, not
        /// the loop's.</para>
        /// </summary>
        public static double[,] Pose(double[] qRad)
        {
            Span<double> t = stackalloc double[16];
            Chain(qRad, t, withTool: true);

            var pose = new double[4, 4];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    pose[i, j] = t[i * 4 + j];

            return pose;
        }

        /// <summary>
        /// Wrist-centre position, metres — the origin of frames 4, 5 and 6.
        /// The three wrist axes meet there, which is what will let the inverse
        /// kinematics split position from orientation. Allocates the array it
        /// returns; use <see cref="WristCentre(double[], double[])"/> in a loop.
        /// </summary>
        public static double[] WristCentre(double[] qRad)
        {
            var p = new double[3];
            WristCentre(qRad, p);
            return p;
        }

        /// <summary>
        /// Wrist centre into a caller-supplied array of three, allocation-free on
        /// the same terms as <see cref="Forward(double[], double[])"/>.
        /// </summary>
        public static void WristCentre(double[] qRad, double[] position)
        {
            if (position.Length < 3)
                throw new ArgumentException(
                    "La posición necesita al menos 3 elementos.", nameof(position));

            Span<double> t = stackalloc double[16];
            Chain(qRad, t, withTool: false);

            position[0] = t[3];
            position[1] = t[7];
            position[2] = t[11];
        }

        // ── Inverse kinematics ───────────────────────────────────────────────

        /// <summary>The θ2 bias from the D-H table, 0.437π. Appears in both directions.</summary>
        private const double ShoulderBias = 0.437 * Math.PI;

        /// <summary>The forearm, which only ever appears as a sum.</summary>
        private const double L34 = L3 + L4;

        /// <summary>
        /// Slack on the reach test, in metres. A target placed exactly on the
        /// workspace boundary lands a few ulps outside it after the arithmetic,
        /// and rejecting that would be wrong; the clamp below absorbs the rest.
        /// </summary>
        private const double ReachSlack = 1e-9;

        /// <summary>
        /// Below this radius the wrist centre sits on the base axis and q1 stops
        /// being defined — the shoulder singularity. Any q1 gives the same point,
        /// so zero is taken.
        /// </summary>
        private const double AxisEps = 1e-9;

        /// <summary>Below this |sin q5| the wrist is singular: q4 and q6 share an axis.</summary>
        private const double WristEps = 1e-9;

        /// <summary>
        /// Joint angles, in model radians, that put the tool tip at
        /// <paramref name="pTip"/> with orientation <paramref name="r06"/>.
        ///
        /// <para><b>Configuration: elbow up, shoulder front, wrist unflipped.</b>
        /// A 6-DoF arm with a spherical wrist has up to eight solutions — shoulder
        /// front or back, elbow up or down, wrist flipped or not. This returns one
        /// of them, deterministically, because a panel target wants one answer and
        /// a controller must never be handed a branch switch: two branches can sit
        /// far apart in joint space for targets a millimetre apart, and taking the
        /// wrong one at the wrong moment is a large commanded motion.</para>
        ///
        /// <para>Returns false with a reason rather than a clamped or NaN pose.
        /// Most hand-typed poses are unreachable, and a silent approximation is
        /// exactly what makes that hard to see.</para>
        ///
        /// <para><b>Both geometric guards apply</b>: reach, that the wrist centre
        /// lands inside the shoulder's annulus, and <see cref="ViperJointLimits"/>,
        /// that every joint the solution asks for is one the linkage allows. What
        /// neither catches is a self-collision, which is not a per-joint
        /// interval.</para>
        /// </summary>
        /// <param name="pTip">Tool tip in the base frame, metres.</param>
        /// <param name="r06">Base-to-tool rotation, 3×3, <c>r06[row, col]</c>.</param>
        /// <param name="qOut">Six joint angles, model radians, written on success.</param>
        /// <param name="reason">Why it failed; empty on success.</param>
        public static bool Inverse(double[] pTip, double[,] r06, double[] qOut, out string reason)
        {
            reason = string.Empty;

            if (pTip.Length < 3)
                { reason = "La posición necesita tres componentes."; return false; }
            if (r06.GetLength(0) < 3 || r06.GetLength(1) < 3)
                { reason = "La orientación necesita una matriz de 3×3."; return false; }
            if (qOut.Length < JointCount)
                { reason = $"El destino necesita {JointCount} articulaciones."; return false; }

            // ── 1. Decouple: back the tip off along the tool axis ─────────────
            //
            // The tool axis is R's third column, and frames 4-6 share the wrist
            // centre, so this one subtraction removes the wrist from the position
            // problem entirely. It is also why position and orientation are not
            // independent: get the orientation wrong and the point you are really
            // solving for moves by the whole tool length.
            double wx = pTip[0] - ToolLength * r06[0, 2];
            double wy = pTip[1] - ToolLength * r06[1, 2];
            double wz = pTip[2] - ToolLength * r06[2, 2];

            // ── 2. q1: the base turn ─────────────────────────────────────────
            double rho = Math.Sqrt(wx * wx + wy * wy);
            double q1  = rho < AxisEps ? 0.0 : Math.Atan2(wy, wx);

            // ── 3. q2 and q3: a two-link planar chain ────────────────────────
            //
            // In the vertical plane q1 just chose, the wrist centre is reached by
            // the upper arm and the forearm alone. Writing the closed form of the
            // forward direction with u = 0.437π − q2 and v = −(q2 + q3) turns it
            // into the textbook chain, because the two D-H biases sum to exactly
            // π/2 and cancel:
            //
            //     ρ = L2·cos u + L34·cos v
            //     h = L2·sin u + L34·sin v
            double h  = wz - L1;
            double d2 = rho * rho + h * h;
            double d  = Math.Sqrt(d2);

            if (d > L2 + L34 + ReachSlack)
            {
                reason = $"Fuera de alcance: el centro de muñeca queda a {d * 100.0:F1} cm " +
                         $"del hombro y el máximo es {(L2 + L34) * 100.0:F1} cm.";
                return false;
            }

            if (d < Math.Abs(L2 - L34) - ReachSlack)
            {
                reason = $"Demasiado cerca: el centro de muñeca queda a {d * 100.0:F1} cm " +
                         $"del hombro y el mínimo es {Math.Abs(L2 - L34) * 100.0:F1} cm.";
                return false;
            }

            double cosBend = Math.Clamp(
                (d2 - L2 * L2 - L34 * L34) / (2.0 * L2 * L34), -1.0, 1.0);

            // The sign is the elbow branch. The elbow sits above the line from
            // shoulder to wrist exactly when sin(bend) < 0 — the cross product of
            // the two reduces to −L2·L34·sin(bend) — so elbow up is the negative
            // root. At the reference pose this gives bend = −0.437π, which is the
            // arm as it stands with every motor centred.
            double bend = -Math.Acos(cosBend);

            double a = L2 + L34 * Math.Cos(bend);
            double b = L34 * Math.Sin(bend);
            double u = Math.Atan2(a * h - b * rho, a * rho + b * h);

            double q2 = ShoulderBias - u;
            double q3 = -bend - ShoulderBias;

            // ── 4. q4, q5, q6: what is left of the rotation ───────────────────
            //
            // R³₆ = (R⁰₃)ᵀ·R⁰₆, and the wrist is a proper Euler set:
            //     R³₆ = Ry(q4)·Rz(q5)·Ry(q6)·Rx(−π/2)
            // so post-multiplying by Rx(π/2) leaves a plain YZY to read off.
            double[,] r03 = RotationToFrame3(q1, q2, q3);

            // m = R³₆·Rx(π/2). Rx(π/2) only permutes columns: column 1 becomes the
            // old column 2, column 2 the negated old column 1.
            var m = new double[3, 3];
            for (int i = 0; i < 3; i++)
            {
                double c0 = 0.0, c1 = 0.0, c2 = 0.0;
                for (int k = 0; k < 3; k++)
                {
                    c0 += r03[k, i] * r06[k, 0];
                    c1 += r03[k, i] * r06[k, 1];
                    c2 += r03[k, i] * r06[k, 2];
                }

                m[i, 0] =  c0;
                m[i, 1] =  c2;
                m[i, 2] = -c1;
            }

            double cq5 = Math.Clamp(m[1, 1], -1.0, 1.0);
            double q5  = Math.Acos(cq5);
            double q4, q6;

            if (Math.Abs(Math.Sin(q5)) < WristEps)
            {
                // The wrist singularity: q4 and q6 turn about the same line, so
                // only their sum or difference is determined. Everything goes to
                // q4, matching what the orientation readout does with φ and ψ.
                q4 = cq5 > 0.0
                    ? Math.Atan2(m[0, 2], m[0, 0])
                    : Math.Atan2(m[0, 2], -m[0, 0]);
                q6 = 0.0;
            }
            else
            {
                q4 = Math.Atan2(m[2, 1], -m[0, 1]);
                q6 = Math.Atan2(m[1, 2],  m[1, 0]);
            }

            // ── 5. Fold into the motors' range, then test the joint limits ────
            //
            // A Dynamixel in position mode spans exactly one turn, so every angle
            // is reachable modulo 2π and folding is not an approximation — 190°
            // and −170° are the same configuration, and only one of the two is a
            // command the motor accepts. Which one is not decided by a symmetric
            // ±180: it depends on where that joint's limits sit, so the fold is
            // done against them. See ViperJointLimits.Fold.
            //
            // The limits are the guard this function lacked. Reach only says the
            // wrist centre lands inside the shoulder's annulus; the shoulder, the
            // elbow and the wrist pitch each stop well short of a full turn, so a
            // pose can satisfy every test above and still be one the linkage
            // refuses. Rejecting it here is the same contract as the reach test —
            // a reason rather than a clamped pose — and it is the last place the
            // rejection is free, because below this the next step is a motor.
            Span<double> q = stackalloc double[JointCount];
            q[0] = q1; q[1] = q2; q[2] = q3; q[3] = q4; q[4] = q5; q[5] = q6;

            for (int i = 0; i < JointCount; i++)
            {
                double folded = ViperJointLimits.Fold(i, Wrap(q[i]));

                if (!ViperJointLimits.Contains(i, folded))
                {
                    reason = "La pose queda dentro del alcance, pero fuera de los " +
                             "topes de la articulación." + Environment.NewLine +
                             Environment.NewLine + ViperJointLimits.Describe(i, folded);
                    return false;
                }

                qOut[i] = folded;
            }

            return true;
        }

        /// <summary>
        /// R⁰₃ for the first three joints, walking the same table the forward
        /// direction walks so the two cannot disagree.
        /// </summary>
        private static double[,] RotationToFrame3(double q1, double q2, double q3)
        {
            Span<double> t    = stackalloc double[16];
            Span<double> link = stackalloc double[16];
            Span<double> next = stackalloc double[16];

            Identity(t);

            Span<double> q = stackalloc double[3];
            q[0] = q1; q[1] = q2; q[2] = q3;

            for (int i = 0; i < 3; i++)
            {
                LinkTransform(Alpha[i], A[i], D[i], q[i] + ThetaOffset[i], link);
                Multiply(t, link, next);
                next.CopyTo(t);
            }

            var r = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    r[i, j] = t[i * 4 + j];

            return r;
        }

        /// <summary>Folds an angle into (−π, π].</summary>
        private static double Wrap(double a)
        {
            double w = Math.IEEERemainder(a, 2.0 * Math.PI);
            return double.IsNegative(w) && w == -Math.PI ? Math.PI : w;
        }

        // ── The chain ────────────────────────────────────────────────────────

        /// <summary>
        /// Walks the D-H table into <paramref name="t"/>, a 4×4 held row-major in
        /// sixteen doubles — <c>t[row * 4 + col]</c>. The caller supplies the
        /// storage, normally <c>stackalloc</c>, which is what keeps every path
        /// through here off the heap.
        /// </summary>
        /// <param name="withTool">
        /// True to reach the tool tip, false to stop at the wrist centre.
        /// </param>
        private static void Chain(double[] qRad, Span<double> t, bool withTool)
        {
            if (qRad.Length < JointCount)
                throw new ArgumentException(
                    $"La cinemática espera {JointCount} ángulos, recibió {qRad.Length}.",
                    nameof(qRad));

            Span<double> link = stackalloc double[16];
            Span<double> next = stackalloc double[16];

            Identity(t);

            for (int i = 0; i < JointCount; i++)
            {
                LinkTransform(Alpha[i], A[i], D[i], qRad[i] + ThetaOffset[i], link);
                Multiply(t, link, next);
                next.CopyTo(t);
            }

            if (!withTool) return;

            // The tool rides on z6, and frames 4, 5 and 6 share the wrist centre,
            // so this one translation is the whole end effector. Post-multiplying
            // by Trans_z(Lt) leaves the rotation alone and adds Lt times its third
            // column — z6 itself — to the position column, so writing that adds
            // the tool without a 4×4 product that is all zeros and ones.
            t[3]  += ToolLength * t[2];
            t[7]  += ToolLength * t[6];
            t[11] += ToolLength * t[10];
        }

        // ── Matrix helpers, all writing into caller-supplied storage ─────────

        /// <summary>
        /// One modified-D-H link transform:
        /// <c>Rot_x(α) · Trans_x(a) · Rot_z(θ) · Trans_z(d)</c>.
        /// </summary>
        private static void LinkTransform(
            double alpha, double a, double d, double theta, Span<double> m)
        {
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha), sa = Math.Sin(alpha);

            m[0]  =      ct;  m[1]  =     -st;  m[2]  = 0.0;  m[3]  =       a;
            m[4]  = st * ca;  m[5]  = ct * ca;  m[6]  = -sa;  m[7]  = -d * sa;
            m[8]  = st * sa;  m[9]  = ct * sa;  m[10] =  ca;  m[11] =  d * ca;
            m[12] =     0.0;  m[13] =     0.0;  m[14] = 0.0;  m[15] =     1.0;
        }

        private static void Identity(Span<double> m)
        {
            m.Clear();
            m[0] = m[5] = m[10] = m[15] = 1.0;
        }

        /// <summary><c>result = a · b</c>. The three must not overlap.</summary>
        private static void Multiply(
            ReadOnlySpan<double> a, ReadOnlySpan<double> b, Span<double> result)
        {
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                {
                    double s = 0.0;
                    for (int k = 0; k < 4; k++) s += a[i * 4 + k] * b[k * 4 + j];
                    result[i * 4 + j] = s;
                }
        }
    }
}
