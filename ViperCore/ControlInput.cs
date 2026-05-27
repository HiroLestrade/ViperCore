namespace ViperCore
{
    public sealed class ControlInput
    {
        public double[] Q    { get; set; } = [];  // current joint angles (rad)
        public double[] Qd   { get; set; } = [];  // desired joint angles from trajectory (rad)
        public double[] Qpf  { get; set; } = [];  // filtered velocity (rad/s)
        public double[] Qppf { get; set; } = [];  // filtered acceleration (rad/s²)
        public double   T    { get; set; }         // time since motion start (s)
        public double   Dt   { get; set; }         // sample time (s)
    }
}
