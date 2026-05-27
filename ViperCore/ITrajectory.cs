namespace ViperCore
{
    public interface ITrajectory
    {
        void Init(double[] q0, double[] qf, double tf);
        void Evaluate(double t, double[] qd);
    }
}
