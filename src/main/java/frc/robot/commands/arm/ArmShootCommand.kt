package frc.robot.commands.arm

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ArmSubsystem

class ArmShootCommand : Command() {
    private val armSubsystem = ArmSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(armSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        armSubsystem.shoot(0.0)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        armSubsystem.stopShooter()
    }
}
