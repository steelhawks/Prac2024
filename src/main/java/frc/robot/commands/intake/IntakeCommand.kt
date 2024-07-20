package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.FeederSubsystem
import frc.robot.subsystems.IntakeSubsystem

class IntakeCommand : Command() {
    private val intakeSubsystem = IntakeSubsystem
    private val feederSubsystem = FeederSubsystem
    private var beamStartBroken = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem, feederSubsystem)
    }

    override fun initialize() {
        beamStartBroken = IntakeSubsystem.intakeBeamBroken
    }

    override fun execute() {
        intakeSubsystem.intake()
        intakeSubsystem.noteStatus = NoteStatus.INTAKING

        if (beamStartBroken) {
            feederSubsystem.feedToShooter()
        }
    }

    override fun isFinished(): Boolean {
        if (beamStartBroken) {
            return false
        }

        return IntakeSubsystem.intakeBeamBroken
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()

        if (beamStartBroken) {
            intakeSubsystem.noteStatus = NoteStatus.IN_SHOOTER
            feederSubsystem.stopFeed()
        } else {
            intakeSubsystem.noteStatus = NoteStatus.NOTHING
        }

        if (!beamStartBroken && IntakeSubsystem.intakeBeamBroken) {
            intakeSubsystem.noteStatus = NoteStatus.INTAKEN
        }
    }
}
