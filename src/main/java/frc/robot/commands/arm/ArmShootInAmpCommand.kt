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
    private var startDegrees = 0.0
    private val degreesToStop = 25.0

    init {
        addRequirements(armSubsystem, elevatorSubsystem)
    }

    override fun initialize() {
        startDegrees = armSubsystem.shooterMotorDegrees
    }

    override fun execute() {
        if (armSubsystem.armInPosition(ArmSubsystem.Position.AMP_SHOOT)) {
            armSubsystem.shoot(true)
//            if (!timer.hasElapsed(shootTime)) {
//                timer.start()
//            }
        }

//        if (timer.hasElapsed(shootTime)) {
//            noteShot = true
//        }
    }

    override fun isFinished(): Boolean {
        return (armSubsystem.shooterMotorDegrees - startDegrees) >= degreesToStop
    }

    override fun end(interrupted: Boolean) {
        armSubsystem.stopShooter()
        armSubsystem.stopArm()
//        timer.stop()
//        timer.reset()

        IntakeSubsystem.noteStatus = NoteStatus.NOTHING
    }
}
