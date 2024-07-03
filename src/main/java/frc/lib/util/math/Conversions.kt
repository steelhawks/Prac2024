package frc.lib.util.math

object Conversions {
    /**
     * @param wheelRPS Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    fun RPSToMPS(wheelRPS: Double, circumference: Double): Double {
        val wheelMPS = wheelRPS * circumference
        return wheelMPS
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    fun MPSToRPS(wheelMPS: Double, circumference: Double): Double {
        val wheelRPS = wheelMPS / circumference
        return wheelRPS
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    fun rotationsToMeters(wheelRotations: Double, circumference: Double): Double {
        val wheelMeters = wheelRotations * circumference
        return wheelMeters
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    fun metersToRotations(wheelMeters: Double, circumference: Double): Double {
        val wheelRotations = wheelMeters / circumference
        return wheelRotations
    }
}