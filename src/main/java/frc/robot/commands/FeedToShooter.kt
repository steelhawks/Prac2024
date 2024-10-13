package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem

class FeedToShooter : ParallelCommandGroup(
    Commands.runEnd(
        {ShooterSubsystem.feedToShooter()},
        {ShooterSubsystem.stopFeed()}
    ),
    ForkCommand(IntakeSubsystem.IntakeDirection.TO_SHOOTER)
)
