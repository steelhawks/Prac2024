package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants

class TeleopDriveCommand(
    private val getLeftY: () -> Double,
    private val getLeftX: () -> Double,
    private val getRightX: () -> Double
) : Command() {
    private val swerveSubsystem = SwerveSubsystem

    init {
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        val leftY = getLeftY()
        val leftX = getLeftX()
        val rightX = getRightX()

        val translationValue = MathUtil.applyDeadband(leftY, Constants.Deadbands.DRIVE_DEADBAND)
        val strafeValue = MathUtil.applyDeadband(leftX,  Constants.Deadbands.DRIVE_DEADBAND)
        val rotationValue = MathUtil.applyDeadband(rightX, Constants.Deadbands.DRIVE_DEADBAND)

        val multipliedTranslation = Translation2d(translationValue, strafeValue).times(Constants.Swerve.MAX_SPEED)
        val multipliedRotation = rotationValue * Constants.Swerve.MAX_ANGULAR_VELOCITY

        swerveSubsystem.drive(
            multipliedTranslation,
            multipliedRotation,
            true
        )
//        println("Executing TeleopDriveCommand with ChassisSpeeds: vx=${newDesiredSpeeds.vxMetersPerSecond}, vy=${newDesiredSpeeds.vyMetersPerSecond}, omega=${newDesiredSpeeds.omegaRadiansPerSecond}")
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        swerveSubsystem.stop()
        println("TeleopDriveCommand ended.")
    }
}
