package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.subsystems.LEDSubsystem

class LEDIdleCommand : Command() {
    private val lEDSubsystem = LEDSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(lEDSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        LEDSubsystem.bounceWave(
            if (RobotContainer.alliance == DriverStation.Alliance.Red) LEDSubsystem.LEDColor.RED else LEDSubsystem.LEDColor.BLUE
        ) // original was green which i think looks better

    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        lEDSubsystem.stop()
    }
}
