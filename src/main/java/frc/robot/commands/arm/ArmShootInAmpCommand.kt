package frc.robot.commands.arm

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.NoteStatus
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.IntakeSubsystem

class ArmShootInAmpCommand : Command() {
    private val armSubsystem = ArmSubsystem
    private val elevatorSubsystem = ElevatorSubsystem

    private var startRotations = 0.0
    private val rotationsToStop = 20

    init {
        addRequirements(armSubsystem, elevatorSubsystem)
    }

    override fun initialize() {
        startRotations = armSubsystem.shooterMotorRotations
    }

    override fun execute() {
        if (armSubsystem.armInPosition(ArmSubsystem.Position.AMP_SHOOT)) {
            armSubsystem.shoot(true)
        }
    }

    override fun isFinished(): Boolean {
        return (armSubsystem.shooterMotorRotations - startRotations) >= rotationsToStop
    }

    override fun end(interrupted: Boolean) {
        armSubsystem.stopShooter()
        armSubsystem.stopArm()

        IntakeSubsystem.noteStatus = NoteStatus.NOTHING
    }
}
