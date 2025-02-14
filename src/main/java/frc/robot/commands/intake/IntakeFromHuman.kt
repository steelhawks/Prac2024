package frc.robot.commands.intake


import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.shooter.RampShooter
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem

class IntakeFromHuman : ParallelCommandGroup(
    RampShooter(-250.0, -250.0, .95),
    Commands.runEnd({
        ShooterSubsystem.feedBackToIntake()
        IntakeSubsystem.intakeReverse()
    },
        {
            ShooterSubsystem.stopFeed()
            IntakeSubsystem.stop()
        }, IntakeSubsystem
    )
        .until(IntakeSubsystem::intakeBeamBroken)
        .andThen(
            SequentialCommandGroup(
                WaitCommand(0.2),
                Commands.runOnce({
                    ShooterSubsystem.stopFeed()
                    IntakeSubsystem.stop()
                }).andThen(IntakeCommand())
            )
        ),
)
