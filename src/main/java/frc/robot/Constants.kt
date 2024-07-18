package frc.robot

import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.geometry.Pose2d
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
    const val PIGEON_CAN_NAME = "canivore"

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
        const val PIGEON_ID = 0

        val CHOSEN_MODULE: COTSTalonFXSwerveConstants =
            COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2)

        /* Drivetrain Constants */
//        val TRACK_WIDTH: Double = Units.inchesToMeters(26.75) // width old
//        val WHEEL_BASE: Double = Units.inchesToMeters(26.75) // length old

        private val TRACK_WIDTH: Double = Units.inchesToMeters(29.5) // width new
        private val WHEEL_BASE: Double = Units.inchesToMeters(30.5) // length new
        val WHEEL_CIRCUMFERENCE: Double = CHOSEN_MODULE.wheelCircumference

        /* Swerve Kinematics */
        val SWERVE_KINEMATICS: SwerveDriveKinematics = SwerveDriveKinematics(
            Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        )

        /* Module Gear Ratios */
        val DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio
        val ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio

        /* Motor Inverts */
        val ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert
        val DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert

        val CANCODER_INVERT: SensorDirectionValue = CHOSEN_MODULE.cancoderInvert

        /* Swerve Current Limiting */
        const val ANGLE_CURRENT_LIMIT = 25
        const val ANGLE_CURRENT_THRESHOLD = 40
        const val ANGLE_CURRENT_THRESHOLD_TIME = 0.1
        const val ANGLE_ENABLE_CURRENT_LIMIT = true

        const val DRIVE_CURRENT_LIMIT = 35
        const val DRIVE_CURRENT_THRESHOLD = 60
        const val DRIVE_CURRENT_THRESHOLD_TIME = 0.1
        const val DRIVE_ENABLE_CURRENT_LIMIT = true

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        const val OPEN_LOOP_RAMP: Double = 0.25
        const val CLOSED_LOOP_RAMP: Double = 0.0

        val ANGLE_KP = CHOSEN_MODULE.angleKP
        val ANGLE_KI = CHOSEN_MODULE.angleKI
        val ANGLE_KD = CHOSEN_MODULE.angleKD

        /* Drive Motor PID Values */
        const val DRIVE_KP = 0.12
        const val DRIVE_KI = 0.0
        const val DRIVE_KD = 0.0
        const val DRIVE_KF = 0.0

        /* Drive Motor Characterization Values From SYSID */
        const val DRIVE_KS = 0.078838
        const val DRIVE_KV = 2.5819
        const val DRIVE_KA = 0.23783

        /* Swerve Profiling Values */
        /** Meters per Second  */
        const val MAX_SPEED: Double = 4.5
        /** Radians per Second  */
        const val MAX_ANGULAR_VELOCITY: Double = 10.0

        val ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast
        val DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake

        const val AUTO_ALIGN_KP = 0.07
        const val AUTO_ALIGN_KI = 0
        const val AUTO_ALIGN_KD = 0
    }
    object Modules {
        object FrontLeft {
            private const val driveMotorID: Int = 1
            private const val angleMotorID: Int = 2
            private const val canCoderID: Int = 3
            private val angleOffset: Rotation2d = Rotation2d.fromDegrees(-130.17)
            val constants: SwerveModuleConstants =
                SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        }

        object FrontRight {
            private const val driveMotorID: Int = 4
            private const val angleMotorID: Int = 5
            private const val canCoderID: Int = 6
            private val angleOffset: Rotation2d = Rotation2d.fromDegrees(-51.34)
            val constants: SwerveModuleConstants =
                SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        }

        object BackLeft {
            private const val driveMotorID: Int = 7
            private const val angleMotorID: Int = 8
            private const val canCoderID: Int = 9
            private val angleOffset: Rotation2d = Rotation2d.fromDegrees(-63.90)
            val constants: SwerveModuleConstants =
                SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        }

        object BackRight {
            private const val driveMotorID: Int = 10
            private const val angleMotorID: Int = 11
            private const val canCoderID: Int = 12
            private val angleOffset: Rotation2d = Rotation2d.fromDegrees(-175.96)
            val constants: SwerveModuleConstants =
                SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        }
    }

    object Deadbands {
        const val DRIVE_DEADBAND: Double = 0.1
        const val ARM_DEADBAND: Double = 0.2
        const val SHOOTER_DEADBAND: Double = 0.2
        const val CLIMB_DEADBAND: Double = 0.4
    }

    object Shooter {
        const val BOTTOM_SHOOTER_MOTOR_ID: Int = 15
        const val TOP_SHOOTER_MOTOR_ID: Int = 16
        const val FEEDER_MOTOR_ID: Int = 17
        const val PIVOT_MOTOR_ID: Int = 18
        const val CANCODER_ID: Int = 19

        const val PIVOT_KS: Double = 0.09256 // 0.09256
        const val PIVOT_KG: Double = 0.15116 // 0.15116
        const val PIVOT_KV: Double = 1.593 // 1.275
        const val PIVOT_KA: Double = 0.0

        const val PIVOT_KP: Double = 5.0
        const val PIVOT_KI: Double = 0.0
        const val PIVOT_KD: Double = 0.0

        const val TOP_SHOOTER_KS: Double = 0.19655
        const val TOP_SHOOTER_KV: Double = 0.00212586
        const val TOP_SHOOTER_KA: Double = 0.00025997

        const val TOP_SHOOTER_KP: Double = 0.01
        const val TOP_SHOOTER_KI: Double = 0.0
        const val TOP_SHOOTER_KD: Double = 0.0

        const val BOTTOM_SHOOTER_KS: Double = 0.13122
        const val BOTTOM_SHOOTER_KV: Double = 0.00198405
        const val BOTTOM_SHOOTER_KA: Double = 0.00071765

        const val BOTTOM_SHOOTER_KP: Double = 0.01
        const val BOTTOM_SHOOTER_KI: Double = 0.0
        const val BOTTOM_SHOOTER_KD: Double = 0.0

        const val K_MAX_VELOCITY_RAD_PER_SECOND: Double = 6.0
        const val K_MAX_ACCELERATION_RAD_PER_SEC_SQUARED: Double = 8.0

        const val HOME_POSITION: Double = 1.05

        const val SHOOTER_TOLERANCE: Double = 50.0

        // testing values
        const val TOP_SHOOTER_SPEED: Double = 0.4
        const val BOTTOM_SHOOTER_SPEED: Double = 0.4
    }

    object LimelightConstants {
        const val LIMELIGHT_SHOOTER: String = "limelight-shooter"
        const val LIMELIGHT_ARM: String = "limelight-arm"

        // Tracking constants
        const val MIN_AREA_OF_TAG: Double = .1
        const val MAX_VISION_POSE_ERROR: Double = 0.5

        // Pipeline IDS
        const val LIMELIGHT_SHOOTER_TAG_PIPELINE: Int = 0
        const val LIMELIGHT_ARM_TAG_PIPELINE: Int = 0
    }

    object PoseConfig {
        // Increase these numbers to trust your model's state estimates less.
        const val K_POSITION_STD_DEV_X: Double = 0.1
        const val K_POSITION_STD_DEV_Y: Double = 0.1
        const val K_POSITION_STD_DEV_THETA: Double = 50.0 // 10

        // Increase these numbers to trust global measurements from vision less.
        const val K_VISION_STD_DEV_X: Double = 5.0
        const val K_VISION_STD_DEV_Y: Double = 5.0
        const val K_VISION_STD_DEV_THETA: Double = Double.MAX_VALUE
    }

    object BlueTeamPoses {
        // Blue team poses
        val INITIAL_POSE: Pose2d = Pose2d(Translation2d(1.225347, 7.652309), Rotation2d()) // Next to blue speaker
        val BLUE_SPEAKER_POSE: Pose2d = Pose2d(Translation2d(0.0, 5.671689), Rotation2d.fromDegrees(180.0))
        val BLUE_ORIGIN: Pose2d = Pose2d(Translation2d(0.0, 0.0), Rotation2d())
        val BLUE_AMP_POSE: Pose2d = Pose2d(Translation2d(1.813129, 8.220855), Rotation2d())
    }

    object RedTeamPoses {
        val initialPose: Pose2d = Pose2d(Translation2d(1.225347, 7.652309), Rotation2d()) // Next to blue speaker
        val RED_ORIGIN: Pose2d = Pose2d(Translation2d(16.542, 8.014), Rotation2d.fromDegrees(180.0))
        val RED_SPEAKER_POSE: Pose2d = Pose2d(Translation2d(16.535595, 5.554168), Rotation2d()).relativeTo(RED_ORIGIN)
        val redAmpPose: Pose2d = Pose2d(Translation2d(14.722884, 8.220855), Rotation2d()).relativeTo(RED_ORIGIN)
    }

    object StructureConstants {
        val SPEAKER_HEIGHT: Double = Units.inchesToMeters(82.0)
        val SHOOTER_ROBOT_HEIGHT: Double = Units.inchesToMeters(14.0)
    }

    object AmpArm {
        const val SHOOTER_MOTOR_ID: Int = 23
        const val PIVOT_MOTOR_ID: Int = 24

        const val CAN_CODER_ID: Int = 25
        const val PIVOT_KS: Double = 0.14
        const val PIVOT_KG: Double = 0.195
        const val PIVOT_KV: Double = 0.85

        const val PIVOT_KP: Double = 1.5
        const val PIVOT_KI: Double = 0.0
        const val PIVOT_KD: Double = 0.0

        const val INTEGRATOR_ZONE: Double = 0.0

        const val PIVOT_TOLERANCE: Double = 0.01

        const val K_MAX_VELOCITY_RAD_PER_SECOND: Double = 13.0
        const val K_MAX_ACCELERATION_RAD_PER_SECOND_SQUARED: Double = 15.0

        const val ARM_OFFSET: Double = -Math.PI / 2

        // Score positions in radians
        const val HOME_POSITION: Double = -2.4
        const val DANGLE_POSITION: Double = -Math.PI / 2
        const val HAND_OFF_POSITION: Double = -2.4
        const val AMP_SLAM_POSITION: Double = -0.20 // NOT USED ANYMORE
        const val CLIMB_IDLE_POSITION: Double = -1.1
        const val AMP_SHOOT_POSITION: Double = -0.55
        const val TRAP_POSITION: Double = 1.37

        // Manual testing values
        const val MANUAL_ARM_PIVOT_SPEED: Double = 0.1
        const val HANDOFF_SPEED: Double = -0.3
        const val SHOOT_SPEED: Double = 0.8
    }
}



