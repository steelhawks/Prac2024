package frc.robot.commands.arm

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.NoteStatus
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.IntakeSubsystem

class ArmHandoffCommand : Command() {
    private val armSubsystem = ArmSubsystem
    private var noteShot = false // goes true when note was shot in amp
    private val timer = Timer()
    private val shootTime = 1.0

    // instead of time change to rotations

    init {
        addRequirements(armSubsystem)
    }

    override fun initialize() {
        noteShot = false
        armSubsystem.goToHandoff()
        timer.reset()
        timer.stop()
    }

    override fun execute() {
        if (armSubsystem.inHandoffPosition) {
            armSubsystem.shoot(0.0)
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
