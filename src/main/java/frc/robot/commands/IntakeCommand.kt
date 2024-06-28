package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem

class IntakeCommand : Command() {
    private val intakeSubsystem = IntakeSubsystem
    private var beamStartBroken = false


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intakeSubsystem)
    }

    override fun initialize() {
        beamStartBroken = IntakeSubsystem.intakeBeamBroken
    }

    override fun execute() {
        IntakeSubsystem.intake()
    }

    override fun isFinished(): Boolean {
        if (beamStartBroken) {
            return false
        }

        return IntakeSubsystem.intakeBeamBroken
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()
    }
}
