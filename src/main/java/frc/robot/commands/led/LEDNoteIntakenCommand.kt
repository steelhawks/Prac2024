package frc.robot.commands.led

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.LEDSubsystem

class LEDNoteIntakenCommand(private val alliance: DriverStation.Alliance) : Command() {
    private val ledSubsystem = LEDSubsystem
    private val color: LEDSubsystem.LEDColor = if (alliance == DriverStation.Alliance.Red) LEDSubsystem.LEDColor.RED else LEDSubsystem.LEDColor.BLUE

    init {
        addRequirements(ledSubsystem)
    }

    override fun initialize() {
        ledSubsystem.setColor(color)
    }

    override fun execute() {
        ledSubsystem.setColor(color)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !IntakeSubsystem.intakeBeamBroken
    }

    override fun end(interrupted: Boolean) {}
}
