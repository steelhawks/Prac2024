package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.SwerveSubsystem


class TeleopDriveCommand(
    private val getLeftY: () -> Double,
    private val getLeftX: () -> Double,
    private val getRightX: () -> Double,
    private val getFieldRelative: () -> Boolean,
    private val getFaceSpeaker: () -> Boolean,
    private val getFaceAmp: () -> Boolean
) : Command() {
    private val swerveSubsystem = SwerveSubsystem

    init {
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {}

    private fun continuous180To360(angle: Double): Double {
        return (angle + 360) % 360
    }

    private fun getRotationSpeedFromPID(target: Pose2d): Double {
        val robotHeading = continuous180To360(SwerveSubsystem.heading.degrees)
        val requestedAngle: Double =
            SwerveSubsystem.calculateTurnAngle(target, SwerveSubsystem.heading.degrees + 180)
        val setpoint = (robotHeading + requestedAngle) % 360

        SwerveSubsystem.alignPID.setSetpoint(setpoint)

        return (if (SwerveSubsystem.isLowGear) 5 else 1) * SwerveSubsystem.alignPID
            .calculate(continuous180To360(SwerveSubsystem.heading.degrees))
    }

    override fun execute() {
        val leftY = getLeftY()
        val leftX = getLeftX()
        val rightX = getRightX()
        val fieldRelative = getFieldRelative()
        val faceSpeaker = getFaceSpeaker()
        val faceAmp = getFaceAmp()

        val translationValue = MathUtil.applyDeadband(leftY, Constants.Deadbands.DRIVE_DEADBAND)
        val strafeValue = MathUtil.applyDeadband(leftX,  Constants.Deadbands.DRIVE_DEADBAND)
        val rotationValue = MathUtil.applyDeadband(rightX, Constants.Deadbands.DRIVE_DEADBAND)

        var multipliedTranslation = Translation2d(translationValue, strafeValue).times(Constants.Swerve.MAX_SPEED)
        var multipliedRotation = rotationValue * Constants.Swerve.MAX_ANGULAR_VELOCITY

        if (faceSpeaker) {
            multipliedRotation = getRotationSpeedFromPID(if (RobotContainer.alliance == DriverStation.Alliance.Blue) Constants.BlueTeamPoses.BLUE_SPEAKER_POSE else Constants.RedTeamPoses.RED_SPEAKER_POSE)
            multipliedTranslation = if (swerveSubsystem.isLowGear) multipliedTranslation else multipliedTranslation.times(0.3)
        } else if (faceAmp) {
            multipliedRotation = getRotationSpeedFromPID(if (RobotContainer.alliance == DriverStation.Alliance.Blue) Constants.BlueTeamPoses.BLUE_AMP_POSE else Constants.RedTeamPoses.RED_AMP_POSE)
        }

        swerveSubsystem.drive(
            multipliedTranslation,
            multipliedRotation,
            fieldRelative,
            true
        )
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        swerveSubsystem.stop()
        println("TeleopDriveCommand ended.")
    }
}
