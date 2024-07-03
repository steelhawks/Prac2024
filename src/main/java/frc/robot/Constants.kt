package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import frc.lib.util.COTSTalonFXSwerveConstants
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

    object Swerve {
        val chosenModule: COTSTalonFXSwerveConstants =
            COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2)
        /* Drivetrain Constants */
        val TRACK_WIDTH: Double = Units.inchesToMeters(26.75) // width
        val WHEEL_BASE: Double = Units.inchesToMeters(26.75) // length
        val WHEEL_CIRCUMFERENCE: Double = chosenModule.wheelCircumference

        /* Swerve Kinematics */
        val SWERVE_KINEMATICS: SwerveDriveKinematics = SwerveDriveKinematics(
            Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        )

        /* Drive Motor PID Values */
        const val driveKP = 0.12
        const val driveKI = 0.0
        const val driveKD = 0.0
        const val driveKF = 0.0

        /* Drive Motor Characterization Values From SYSID */
        const val DRIVE_KS = 0.078838
        const val DRIVE_KV = 2.5819
        const val DRIVE_KA = 0.23783

        const val MAX_SPEED: Double = 4.5
    }
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



