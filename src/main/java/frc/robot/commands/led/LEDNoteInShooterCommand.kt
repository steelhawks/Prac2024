package frc.robot.commands.led

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.LEDSubsystem

class LEDNoteInShooterCommand(private val color: LEDSubsystem.LEDColor) : Command() {
    private val ledSubsystem = LEDSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(ledSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        ledSubsystem.fade(color)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return IntakeSubsystem.noteStatus != NoteStatus.IN_SHOOTER
    }

    override fun end(interrupted: Boolean) {}
}
