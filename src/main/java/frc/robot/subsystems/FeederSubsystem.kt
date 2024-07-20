package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants

object FeederSubsystem : SubsystemBase() {
    private val m_feederMotor: TalonFX = TalonFX(Constants.Shooter.FEEDER_MOTOR_ID, Constants.CANIVORE_NAME)

    init {
        m_feederMotor.setNeutralMode(NeutralModeValue.Coast)
    }

    fun feedToShooter() {
        m_feederMotor.set(10.0)
    }

    fun feedBackToIntake() {
        m_feederMotor.set(-0.5)
    }

    fun stopFeed() {
        m_feederMotor.stopMotor()
    }
}