package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem

class RotateToAngleCommand() : Command() {
    private val swerveSubsystem = SwerveSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {}

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        swerveSubsystem.stop()
    }
}
