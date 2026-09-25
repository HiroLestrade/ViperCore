namespace ViperCore
{
    /// <summary>
    /// El puente entre los dos convenios de ángulo que conviven dentro de esta
    /// biblioteca.
    ///
    /// <list type="bullet">
    /// <item><b>Grados Dynamixel</b> — 0 a 360 con 180° en el centro. Lo que
    /// reportan los motores y lo que habla <see cref="ViperDevice"/>.</item>
    /// <item><b>Radianes del modelo</b> — la q del artículo, cero en la
    /// configuración de referencia. Lo que hablan
    /// <see cref="ViperKinematics"/> y <see cref="ViperDynamics"/>.</item>
    /// </list>
    ///
    /// <para>Sólo difieren por el centro y el factor grado-radián:
    /// <c>q = (dxl − 180)·π/180</c>. Los sesgos D-H de q2 y q3 <b>no</b> están
    /// aquí: viven dentro de la tabla, en cada modelo, para aplicarse una vez.</para>
    ///
    /// <para>Existía sólo en RobotClient, que es donde la interfaz la necesita.
    /// Vive aquí desde que el paro de emergencia en modo corriente tuvo que
    /// evaluar la gravedad, porque eso puso a <see cref="ViperDevice"/> a
    /// necesitar la conversión sin poder subir a buscarla.</para>
    ///
    /// <para><b>La dirección está verificada, pero sólo para la posición</b>: se
    /// toma una lectura Dynamixel creciente como una q creciente en las seis
    /// articulaciones, y eso se comprobó con el brazo —la prueba de la cinta
    /// métrica y el modo cartesiano dan lo que predice la cinemática—. Lo que
    /// <b>no</b> se ha comprobado es el mismo supuesto aplicado al <b>par</b>: que
    /// un `Goal Current` positivo empuje hacia q creciente. Es la propiedad del
    /// actuador, no la del encoder, y sigue siendo la causa probable del incidente
    /// del 23 de septiembre.</para>
    /// </summary>
    public static class ViperAngles
    {
        /// <summary>Grados Dynamixel en el centro del motor, donde la q del modelo es cero.</summary>
        public const double CentreDeg = 180.0;

        private const double DegToRad = Math.PI / 180.0;
        private const double RadToDeg = 180.0 / Math.PI;

        public static double ToModelRad(double dxlDeg) => (dxlDeg - CentreDeg) * DegToRad;

        public static double ToDynamixelDeg(double qRad) => qRad * RadToDeg + CentreDeg;

        /// <summary>Grados Dynamixel a radianes del modelo, en un arreglo del llamador.</summary>
        public static void ToModelRad(double[] dxlDeg, double[] qRad)
        {
            for (int i = 0; i < dxlDeg.Length && i < qRad.Length; i++)
                qRad[i] = ToModelRad(dxlDeg[i]);
        }

        /// <summary>Grados Dynamixel a radianes del modelo. Asigna.</summary>
        public static double[] ToModelRad(double[] dxlDeg)
        {
            var q = new double[dxlDeg.Length];
            ToModelRad(dxlDeg, q);
            return q;
        }

        /// <summary>
        /// Velocidades: deg/s a rad/s. <b>Sin desplazar el centro</b> — una
        /// diferencia entre dos ángulos vale lo mismo en los dos convenios, así
        /// que aquí sólo cambia la unidad.
        /// </summary>
        public static void RateToModelRad(double[] degPerSec, double[] radPerSec)
        {
            for (int i = 0; i < degPerSec.Length && i < radPerSec.Length; i++)
                radPerSec[i] = degPerSec[i] * DegToRad;
        }
    }
}
