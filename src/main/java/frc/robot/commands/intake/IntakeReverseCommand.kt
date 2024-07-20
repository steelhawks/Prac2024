package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.FeederSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem

class IntakeReverseCommand : Command() {
    private val intakeSubsystem = IntakeSubsystem
    private val feederSubsystem = FeederSubsystem
    private val shooterSubsystem = ShooterSubsystem

    init {
        addRequirements(intakeSubsystem, feederSubsystem, shooterSubsystem)
    }

    override fun initialize() {}

    override fun execute() {
        intakeSubsystem.intakeReverse()

        if (intakeSubsystem.noteStatus == NoteStatus.IN_SHOOTER) {
            feederSubsystem.feedBackToIntake()
            shooterSubsystem.reverseToIntake()
        }


        if (intakeSubsystem.intakeBeamBroken) {
            intakeSubsystem.noteStatus = NoteStatus.INTAKEN
        }
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.stop()
        feederSubsystem.stopFeed()
        shooterSubsystem.stopShooter()
    }
}
