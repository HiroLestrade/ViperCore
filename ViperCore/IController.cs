namespace ViperCore
{
    public interface IController
    {
        void Reset();
        void Compute(ControlInput input, double[] output);
    }
}
