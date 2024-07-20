package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem

class IntakeToArmCommand : Command() {
    private val armSubsystem = ArmSubsystem
    private val intakeSubsystem = IntakeSubsystem

    private var prevBeamBroken = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(armSubsystem, intakeSubsystem)
    }

    override fun initialize() {
        intakeSubsystem.noteStatus = NoteStatus.INTAKING

        prevBeamBroken = false
    }

    override fun execute() {
        intakeSubsystem.intakeToArm()

        armSubsystem.shoot(false)

        if (intakeSubsystem.armBeamBroken) {
            prevBeamBroken = true
        }
    }

    override fun isFinished(): Boolean {
        return prevBeamBroken && !IntakeSubsystem.armBeamBroken
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()
        armSubsystem.stopShooter()

        if (prevBeamBroken)
            intakeSubsystem.noteStatus = NoteStatus.ARM
        else
            intakeSubsystem.noteStatus = NoteStatus.INTAKEN
    }
}
