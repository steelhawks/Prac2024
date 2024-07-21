package frc.robot.commands.elevator


import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.ElevatorSubsystem

// TODO: Add your sequential commands in the super constructor call,
//       e.g. SequentialCommandGroup(OpenClawCommand(), MoveArmCommand())
class ElevatorHomePositionCommand : SequentialCommandGroup(
    Commands.runOnce(ElevatorSubsystem::disable).andThen(
        Commands.run({
            ElevatorSubsystem.controlElevator(true)
        })
            .until(ElevatorSubsystem::atElevatorMin)
                .andThen(ElevatorSubsystem::stopElevator)
    )
)