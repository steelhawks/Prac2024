package frc.lib.util.math

import edu.wpi.first.math.geometry.Translation2d

object MathConstants {
    val TRANSLATION2D_ZERO = Translation2d(0.0, 0.0)
    const val ROTATION_ZERO = 0.0


    fun continuous180To360(angle: Double): Double {
        return (angle + 360) % 360
    }
}
