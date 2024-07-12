package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem

class IntakeReverseCommand : Command() {
    private val intakeSubsystem = IntakeSubsystem

    init {
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        intakeSubsystem.intakeReverse()
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()
    }
}
