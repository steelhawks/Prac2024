package frc.robot.commands.arm

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.ArmSubsystem

class ArmHomePositionCommand : Command() {
    private val armSubsystem = ArmSubsystem

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(armSubsystem)
    }

    override fun initialize() {
//        armSubsystem.setGoal(Constants.AmpArm.HOME_POSITION)
        armSubsystem.goHome()
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
//        return !armSubsystem.controller.atGoal()
        return false
    }

    override fun end(interrupted: Boolean) {
        println("Ended properly")
        armSubsystem.stopArm()
    }
}
