package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants

object ShooterSubsystem : SubsystemBase() {
    private val m_pivotMotor: TalonFX = TalonFX(Constants.Shooter.PIVOT_MOTOR_ID, Constants.CANIVORE_NAME)
    private val m_bottomShooterMotor: TalonFX = TalonFX(Constants.Shooter.BOTTOM_SHOOTER_MOTOR_ID, Constants.CANIVORE_NAME)
    private val m_topShooterMotor: TalonFX = TalonFX(Constants.Shooter.TOP_SHOOTER_MOTOR_ID, Constants.CANIVORE_NAME)

    private val m_canCoder = CANcoder(Constants.Shooter.CANCODER_ID, Constants.CANIVORE_NAME)

    private val bottomShooterPIDController: PIDController =
        PIDController(
            Constants.Shooter.BOTTOM_SHOOTER_KP,
            Constants.Shooter.BOTTOM_SHOOTER_KI,
            Constants.Shooter.BOTTOM_SHOOTER_KD
        )

    private val topShooterPIDController: PIDController =
        PIDController(
            Constants.Shooter.TOP_SHOOTER_KP,
            Constants.Shooter.TOP_SHOOTER_KI,
            Constants.Shooter.TOP_SHOOTER_KD
        )

    // shooter feedforwards
    private val topShooterFeedForward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(
            Constants.Shooter.TOP_SHOOTER_KS,
            Constants.Shooter.TOP_SHOOTER_KV
        )

    private val bottomShooterFeedForward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(
            Constants.Shooter.BOTTOM_SHOOTER_KS,
            Constants.Shooter.BOTTOM_SHOOTER_KV
        )

    // pivot feedforward
    private val pivotFeedForward: ArmFeedforward =
        ArmFeedforward(
            Constants.Shooter.PIVOT_KS,
            Constants.Shooter.PIVOT_KG,
            Constants.Shooter.PIVOT_KV
        )

    init {
        

        configureMotors()
    }

    private fun configureMotors() {
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake)
        m_bottomShooterMotor.setNeutralMode(NeutralModeValue.Brake)
        m_topShooterMotor.setNeutralMode(NeutralModeValue.Brake)

        m_pivotMotor.inverted = false
        m_pivotMotor.setPosition(0.0)
    }

    fun stopPivot() {
        m_topShooterMotor.stopMotor()
    }
}