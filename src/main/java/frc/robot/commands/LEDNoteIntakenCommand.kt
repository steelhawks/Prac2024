import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.LEDSubsystem

class LEDNoteIntakenCommand : Command() {

    init {
        addRequirements(LEDSubsystem, IntakeSubsystem)
    }

    override fun initialize() {
        // Flash the LEDs for a specified duration
        LEDSubsystem.flashCommand(LEDSubsystem.LEDColor.GREEN, 0.1, 1.0)
            .andThen(InstantCommand ({
                LEDSubsystem.setColor(LEDSubsystem.LEDColor.GREEN)
            }))
    }

    override fun execute() {
        // No additional execution logic required
    }

    override fun isFinished(): Boolean {
        return !IntakeSubsystem.intakeBeamBroken
    }

    override fun end(interrupted: Boolean) {
        LEDSubsystem.setColor(LEDSubsystem.LEDColor.GREEN)
    }
}
