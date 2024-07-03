package frc.robot.commands

import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command

class TeleopDriveCommand(
    private val getLeftX: () -> Double,
    private val getLeftY: () -> Double,
    private val getRightX: () -> Double
) : Command() {
    private val swerveSubsystem = SwerveSubsystem

    init {
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        val newDesiredSpeeds = ChassisSpeeds(
            getLeftY(),
            getLeftX(),
            getRightX()
        )

        swerveSubsystem.setChassisSpeed(newDesiredSpeeds)
//        println("Executing TeleopDriveCommand with ChassisSpeeds: vx=${newDesiredSpeeds.vxMetersPerSecond}, vy=${newDesiredSpeeds.vyMetersPerSecond}, omega=${newDesiredSpeeds.omegaRadiansPerSecond}")
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        swerveSubsystem.stop()
        println("TeleopDriveCommand ended.")
    }
}
