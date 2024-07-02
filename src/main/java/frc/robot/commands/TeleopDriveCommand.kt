package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import SwerveSubsystem
import edu.wpi.first.math.kinematics.ChassisSpeeds

class TeleopDriveCommand(private val getLeftX: () -> Double, private val getLeftY: () -> Double, private val getRightX: () -> Double) : Command() {
    private val swerveSubsystem = SwerveSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        val newDesiredSpeeds = ChassisSpeeds(
            getLeftY(),
            getLeftX(),
            getRightX(),
        )

        swerveSubsystem.setChassisSpeed(newDesiredSpeeds)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
