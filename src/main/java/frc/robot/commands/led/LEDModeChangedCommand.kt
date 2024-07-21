package frc.robot.commands.led

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LEDSubsystem

class LEDModeChangedCommand : Command() {
    private val lEDSubsystem = LEDSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(lEDSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        lEDSubsystem.pulse(LEDSubsystem.LEDColor.GREEN, .5)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
