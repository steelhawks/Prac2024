package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants

object ShooterSubsystem : SubsystemBase() {
    private val m_pivotMotor: TalonFX = TalonFX(Constants.Shooter.PIVOT_MOTOR_ID, Constants.CANIVORE_NAME)
    private val m_bottomShooterMotor: TalonFX = TalonFX(Constants.Shooter.BOTTOM_SHOOTER_MOTOR_ID, Constants.CANIVORE_NAME)
    private val m_topShooterMotor: TalonFX = TalonFX(Constants.Shooter.TOP_SHOOTER_MOTOR_ID, Constants.CANIVORE_NAME)

    private val m_canCoder = CANcoder(Constants.Shooter.CANCODER_ID, Constants.CANIVORE_NAME)

    init {
        configureMotors()
    }

    private fun configureMotors() {
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake)
        m_bottomShooterMotor.setNeutralMode(NeutralModeValue.Brake)
        m_topShooterMotor.setNeutralMode(NeutralModeValue.Brake)

        m_pivotMotor.setPosition(0.0)
    }
}