package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.FeederSubsystem

class FeederTestCommand : Command() {
    private val feederSubsystem = FeederSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(feederSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        feederSubsystem.feederTest()
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        feederSubsystem.stopFeed()
    }
}
