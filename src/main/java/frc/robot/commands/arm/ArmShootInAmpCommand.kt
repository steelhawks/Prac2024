package frc.robot.commands.arm

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.NoteStatus
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.IntakeSubsystem

class ArmShootInAmpCommand : Command() {
    private val armSubsystem = ArmSubsystem
    private val elevatorSubsystem = ElevatorSubsystem
    private var noteShot = false // goes true when note was shot in amp
    private val timer = Timer()
    private val shootTime = .25

    // instead of time change to rotations

    init {
        addRequirements(armSubsystem, elevatorSubsystem)
    }

    override fun initialize() {
        noteShot = false
        armSubsystem.goToAmpFirePosition()
        Commands.runOnce(elevatorSubsystem::getAmpCommand)
        timer.reset()
        timer.stop()
    }

    override fun execute() {
        if (armSubsystem.inAmpFirePosition) {
//            armSubsystem.shoot(true)
            if (!timer.hasElapsed(shootTime)) {
                timer.start()
            }
        }

        if (timer.hasElapsed(shootTime)) {
            noteShot = true
        }
    }

    override fun isFinished(): Boolean {
        return noteShot
    }

    override fun end(interrupted: Boolean) {
        armSubsystem.stopShooter()
        armSubsystem.stopArm()
        timer.stop()
        timer.reset()

        IntakeSubsystem.noteStatus = NoteStatus.NOTHING
    }
}
