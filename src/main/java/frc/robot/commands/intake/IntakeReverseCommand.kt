package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.FeederSubsystem
import frc.robot.subsystems.IntakeSubsystem

class IntakeReverseCommand : Command() {
    private val intakeSubsystem = IntakeSubsystem
    private val feederSubsystem = FeederSubsystem

    init {
        addRequirements(intakeSubsystem, feederSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        intakeSubsystem.intakeReverse()
        feederSubsystem.feedBackToIntake()

        if (intakeSubsystem.intakeBeamBroken) {
            intakeSubsystem.noteStatus = NoteStatus.INTAKEN
        }
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()
        feederSubsystem.stopFeed()
    }
}
