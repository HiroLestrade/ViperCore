namespace ViperCore
{
    /// <summary>
    /// Fifth-order polynomial trajectory. Guarantees zero velocity and acceleration
    /// at both endpoints. Identical math to GeomagicCore's PolyTrajectory.
    /// </summary>
    public sealed class PolyTrajectory : ITrajectory
    {
        private double   _tf;
        private double[] _qf   = [];
        private double[] _a0   = [];
        private double[] _a3   = [];
        private double[] _a4   = [];
        private double[] _a5   = [];

        public void Init(double[] q0, double[] qf, double tf)
        {
            int n = q0.Length;
            _tf = tf;
            _qf = (double[])qf.Clone();
            _a0 = (double[])q0.Clone();
            _a3 = new double[n];
            _a4 = new double[n];
            _a5 = new double[n];

            for (int i = 0; i < n; i++)
            {
                double d = qf[i] - q0[i];
                _a3[i] =  10.0 * d / (tf * tf * tf);
                _a4[i] = -15.0 * d / (tf * tf * tf * tf);
                _a5[i] =   6.0 * d / (tf * tf * tf * tf * tf);
            }
        }

        public void Evaluate(double t, double[] qd)
        {
            for (int i = 0; i < qd.Length; i++)
            {
                qd[i] = t <= _tf
                    ? _a0[i] + _a3[i]*t*t*t + _a4[i]*t*t*t*t + _a5[i]*t*t*t*t*t
                    : _qf[i];
            }
        }
    }
}
