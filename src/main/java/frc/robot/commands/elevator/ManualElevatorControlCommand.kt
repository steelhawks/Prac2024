package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem

class ManualElevatorControlCommand(private val isDown: () -> Boolean) : Command() {
    private val elevatorSubsystem = ElevatorSubsystem
    private val armSubsystem = ArmSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(elevatorSubsystem, armSubsystem)
    }

    override fun initialize() {
        armSubsystem.goToDangle()
    }

    override fun execute() {
        if (!armSubsystem.armInPosition(ArmSubsystem.Position.DANGLE))
            return

        val down = isDown()
        elevatorSubsystem.controlElevator(down)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        println("Ended")
        elevatorSubsystem.stopElevator()
    }
}
