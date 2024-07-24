package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ShooterSubsystem

open class RampShooter(val topRPM: Double, val bottomRPM: Double, val angle: Double) : Command() {
    private val shooterSubsystem = ShooterSubsystem


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(shooterSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        shooterSubsystem.rampShooter(topRPM, bottomRPM)
        shooterSubsystem.setGoal(angle)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.stopShooter()
    }
}