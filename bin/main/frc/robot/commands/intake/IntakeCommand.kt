package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.LEDSubsystem

class IntakeCommand : Command() {
    private val intakeSubsystem = IntakeSubsystem
    private var beamStartBroken = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        beamStartBroken = IntakeSubsystem.intakeBeamBroken
    }

    override fun execute() {
        IntakeSubsystem.intake()
    }

    override fun isFinished(): Boolean {
        if (beamStartBroken) {
            return false
        }

        return IntakeSubsystem.intakeBeamBroken
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.noteStatus = NoteStatus.INTAKEN
        intakeSubsystem.stop()
    }
}
