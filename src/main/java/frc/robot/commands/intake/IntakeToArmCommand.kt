package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem

class IntakeToArmCommand : Command() {
    private val armSubsystem = ArmSubsystem
    private val intakeSubsystem = IntakeSubsystem

    private var prevBreamBroken = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(armSubsystem, intakeSubsystem)
    }

    override fun initialize() {
        intakeSubsystem.noteStatus = NoteStatus.INTAKING

        prevBreamBroken = IntakeSubsystem.armBeamBroken
    }

    override fun execute() {
        intakeSubsystem.intakeToArm()

        armSubsystem.shoot(1.0) // test val
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if (prevBreamBroken) {
            return !IntakeSubsystem.armBeamBroken
        }

        return IntakeSubsystem.armBeamBroken
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()
        armSubsystem.stopShooter()

        if (prevBreamBroken)
            intakeSubsystem.noteStatus = NoteStatus.ARM
        else
            intakeSubsystem.noteStatus = NoteStatus.INTAKEN
    }
}
