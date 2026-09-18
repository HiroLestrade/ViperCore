namespace ViperCore
{
    /// <summary>
    /// Fifth-order polynomial trajectory: zero velocity and acceleration at both
    /// endpoints. Same math as GeomagicCore's PolyTrajectory, extended to return
    /// the two derivatives analytically rather than leaving the controller to
    /// differentiate the reference.
    ///
    ///   qd   = a0 + a3 t³ +  a4 t⁴ +  a5 t⁵
    ///   qpd  =      3a3 t² + 4a4 t³ + 5a5 t⁴
    ///   qppd =      6a3 t  +12a4 t² +20a5 t³
    /// </summary>
    public sealed class PolyTrajectory : ITrajectory
    {
        private double   _tf;
        private double[] _qf = [];
        private double[] _a0 = [];
        private double[] _a3 = [];
        private double[] _a4 = [];
        private double[] _a5 = [];

        public void Init(double[] q0, double[] qf, double tf)
        {
            int n = q0.Length;
            _tf = tf > 0.0 ? tf : 1e-6;
            _qf = (double[])qf.Clone();
            _a0 = (double[])q0.Clone();
            _a3 = new double[n];
            _a4 = new double[n];
            _a5 = new double[n];

            double t3 = _tf * _tf * _tf;
            for (int i = 0; i < n; i++)
            {
                double d = qf[i] - q0[i];
                _a3[i] =  10.0 * d / t3;
                _a4[i] = -15.0 * d / (t3 * _tf);
                _a5[i] =   6.0 * d / (t3 * _tf * _tf);
            }
        }

        public void Evaluate(double t, double[] qd, double[]? qpd, double[]? qppd)
        {
            bool done = t > _tf;
            double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

            for (int i = 0; i < qd.Length; i++)
            {
                if (done)
                {
                    qd[i] = _qf[i];
                    if (qpd  != null) qpd[i]  = 0.0;
                    if (qppd != null) qppd[i] = 0.0;
                    continue;
                }

                qd[i] = _a0[i] + _a3[i] * t3 + _a4[i] * t4 + _a5[i] * t5;
                if (qpd  != null) qpd[i]  = 3.0 * _a3[i] * t2 + 4.0 * _a4[i] * t3 + 5.0 * _a5[i] * t4;
                if (qppd != null) qppd[i] = 6.0 * _a3[i] * t  + 12.0 * _a4[i] * t2 + 20.0 * _a5[i] * t3;
            }
        }
    }
}
