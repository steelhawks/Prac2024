package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem
import frc.robot.Constants

object ShooterSubsystem : ProfiledPIDSubsystem(
    ProfiledPIDController(
        Constants.Shooter.PIVOT_KP,
        Constants.Shooter.PIVOT_KI,
        Constants.Shooter.PIVOT_KD,
        TrapezoidProfile.Constraints(
            Constants.Shooter.K_MAX_VELOCITY_RAD_PER_SECOND,
            Constants.Shooter.K_MAX_ACCELERATION_RAD_PER_SEC_SQUARED
        ),
        Constants.Shooter.HOME_POSITION,
    )
) {
    private val m_pivotMotor: TalonFX = TalonFX(Constants.Shooter.PIVOT_MOTOR_ID, Constants.CANIVORE_NAME)
    private val m_bottomShooterMotor: TalonFX =
        TalonFX(Constants.Shooter.BOTTOM_SHOOTER_MOTOR_ID, Constants.CANIVORE_NAME)
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

    var trigTargetAngle: Double = 0.0

    init {
        topShooterPIDController.setTolerance(Constants.Shooter.SHOOTER_TOLERANCE)
        bottomShooterPIDController.setTolerance(Constants.Shooter.SHOOTER_TOLERANCE)

        m_bottomShooterMotor.setVoltage(trigTargetAngle)
        m_pivotMotor.inverted = false
        configureMotors()
        enable()
    }

    private fun configureMotors() {
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake)
        m_bottomShooterMotor.setNeutralMode(NeutralModeValue.Brake)
        m_topShooterMotor.setNeutralMode(NeutralModeValue.Brake)

        m_pivotMotor.inverted = false
        m_pivotMotor.setPosition(0.0)
    }

    fun goHome() {
        setGoal(Constants.Shooter.HOME_POSITION)
    }

    fun stopPivot() {
        m_topShooterMotor.stopMotor()
    }

    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State?) {
        println("Use Output")
    }

    override fun getMeasurement(): Double {
        println("Get Measurement")
        return 0.0
    }
}