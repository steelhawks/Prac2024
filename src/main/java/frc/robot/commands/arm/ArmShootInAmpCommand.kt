package frc.robot.commands.arm

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.IntakeSubsystem

class ArmShootInAmpCommand : Command() {
    private val armSubsystem = ArmSubsystem
    private val elevatorSubsystem = ElevatorSubsystem

    // instead of time change to rotations
    private var startDegrees = 0.0
    private val degreesToStop = 415

    init {
        addRequirements(armSubsystem, elevatorSubsystem)
    }

    override fun initialize() {
        startDegrees = armSubsystem.shooterMotorDegrees
    }

    override fun execute() {
        if (armSubsystem.armInPosition(ArmSubsystem.Position.AMP_SHOOT)) {
            armSubsystem.shoot(true)
        }
    }

    override fun isFinished(): Boolean {
        return (armSubsystem.shooterMotorDegrees - startDegrees) >= degreesToStop
    }

    override fun end(interrupted: Boolean) {
        armSubsystem.stopShooter()
        armSubsystem.stopArm()

        IntakeSubsystem.noteStatus = NoteStatus.NOTHING
    }
}
