package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.utils.SwerveModuleConstants


/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object Constants {
    const val CANIVORE_NAME = "canivore"

    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
        const val OPERATOR_CONTROLLER_PORT = 1
    }

    object LED {
        const val PORT = 0
        const val LENGTH = 40
    }

    object Intake {
        const val INTAKE_MOTOR_1_ID: Int = 13
        const val INTAKE_MOTOR_2_ID: Int = 26
        const val FORK_MOTOR_ID: Int = 14

        const val BEAM_BREAKER_INTAKE: Int = 8
        const val BEAM_BREAKER_ARM: Int = 9

        const val INTAKE_VOLTAGE: Double = 7.0
        const val FORK_VOLTAGE: Double = 7.0

        // Manual Values
        const val INTAKE_SPEED: Double = 1.0
        const val FORK_SPEED: Double = 1.0
        const val INTAKE_FEED_SPEED: Double = 1.0 // for arm and shooter
    }

    // Swerve Modules
    object Modules {
        object FrontLeft {
            const val driveMotorID: Int = 1
            const val angleMotorID: Int = 2
            const val canCoderID: Int = 3
            val angleOffset: Rotation2d = Rotation2d.fromDegrees(-130.17)
            val constants: SwerveModuleConstants =
                SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        }

        object FrontRight {
            const val driveMotorID: Int = 4
            const val angleMotorID: Int = 5
            const val canCoderID: Int = 6
            val angleOffset: Rotation2d = Rotation2d.fromDegrees(-51.34)
            val constants: SwerveModuleConstants =
                SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        }

        object BackLeft {
            const val driveMotorID: Int = 7
            const val angleMotorID: Int = 8
            const val canCoderID: Int = 9
            val angleOffset: Rotation2d = Rotation2d.fromDegrees(-63.90)
            val constants: SwerveModuleConstants =
                SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        }

        object BackRight {
            const val driveMotorID: Int = 10
            const val angleMotorID: Int = 11
            const val canCoderID: Int = 12
            val angleOffset: Rotation2d = Rotation2d.fromDegrees(-175.96)
            val constants: SwerveModuleConstants =
                SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        }
    }
}


