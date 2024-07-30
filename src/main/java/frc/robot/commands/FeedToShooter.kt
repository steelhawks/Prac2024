package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ShooterSubsystem

class FeedToShooter : Command() {
    private val shooterSubsystem = ShooterSubsystem

    override fun initialize() {}

    override fun execute() {
        shooterSubsystem.feedToShooter()
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.stopFeed()
    }
}
