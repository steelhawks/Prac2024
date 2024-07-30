package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.FeederSubsystem
import frc.robot.subsystems.IntakeSubsystem

class IntakeCommand : Command() {
    private val intakeSubsystem = IntakeSubsystem
    private val feederSubsystem = FeederSubsystem
    private var prevBeamBroken = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem, feederSubsystem)
    }

    override fun initialize() {
        prevBeamBroken = IntakeSubsystem.intakeBeamBroken
    }

    override fun execute() {
        intakeSubsystem.intake()
        intakeSubsystem.noteStatus = NoteStatus.INTAKING

        if (prevBeamBroken) {
            feederSubsystem.feedToShooter()
        }
    }

    override fun isFinished(): Boolean {
        if (prevBeamBroken) {
            return false
        }

        return IntakeSubsystem.intakeBeamBroken
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()

        if (prevBeamBroken) {
            intakeSubsystem.noteStatus = NoteStatus.IN_SHOOTER
            feederSubsystem.stopFeed()
        } else {
            intakeSubsystem.noteStatus = NoteStatus.NOTHING
        }

        if (!prevBeamBroken && IntakeSubsystem.intakeBeamBroken) {
            intakeSubsystem.noteStatus = NoteStatus.INTAKEN
        }
    }
}
