package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem
import frc.robot.Constants
import kotlin.math.abs

object ArmSubsystem : ProfiledPIDSubsystem(
    ProfiledPIDController(
        Constants.AmpArm.PIVOT_KP,
        Constants.AmpArm.PIVOT_KI,
        Constants.AmpArm.PIVOT_KD,
        TrapezoidProfile.Constraints(
            Constants.AmpArm.K_MAX_VELOCITY_RAD_PER_SECOND,
            Constants.AmpArm.K_MAX_ACCELERATION_RAD_PER_SECOND_SQUARED)),
    Constants.AmpArm.HOME_POSITION
) {
    private val m_pivotMotor = TalonFX(Constants.AmpArm.PIVOT_MOTOR_ID, Constants.CANIVORE_NAME)
    private val m_shootMotor = TalonFX(Constants.AmpArm.SHOOTER_MOTOR_ID, Constants.CANIVORE_NAME)
    private val m_canCoder = CANcoder(Constants.AmpArm.CAN_CODER_ID, Constants.CANIVORE_NAME)

    enum class Position(val rotations: Double) {
        HOME(Constants.AmpArm.HOME_POSITION),
        DANGLE(Constants.AmpArm.DANGLE_POSITION),
        HANDOFF(Constants.AmpArm.HAND_OFF_POSITION),
        AMP_SLAM(Constants.AmpArm.AMP_SLAM_POSITION),
        AMP_SHOOT(Constants.AmpArm.AMP_SHOOT_POSITION),
        CLIMB_POSITION(Constants.AmpArm.CLIMB_IDLE_POSITION),
        TRAP(Constants.AmpArm.TRAP_POSITION)
    }

    enum class ArmStatus {
        NOTHING,
        INTAKING,
    }

    val status = ArmStatus.NOTHING

    private val m_feedForward = ArmFeedforward(
        Constants.AmpArm.PIVOT_KS,
        Constants.AmpArm.PIVOT_KG,
        Constants.AmpArm.PIVOT_KV,
    )

    init {
        controller.setTolerance(Constants.AmpArm.PIVOT_TOLERANCE)
        controller.iZone = Constants.AmpArm.INTEGRATOR_ZONE
        controller.enableContinuousInput(0.0, Math.PI * 2.0)

        enable()
        configureMotors()
    }

    private fun configureMotors() {
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake)
        m_shootMotor.setNeutralMode(NeutralModeValue.Coast)

        m_pivotMotor.setPosition(-90.0 / 14.7)
    }

    fun shoot(towardsAmp: Boolean) {
        if (towardsAmp) {
            m_shootMotor.set(Constants.AmpArm.SHOOT_SPEED)
        } else {
            m_shootMotor.set(Constants.AmpArm.HANDOFF_SPEED)
        }
    }

    fun reverseToIntake() {
        m_shootMotor.set(-Constants.AmpArm.HANDOFF_SPEED)
    }

    fun stopShooter() {
        m_shootMotor.stopMotor()
    }

    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State?) {
        val feedForward = setpoint?.let { m_feedForward.calculate(it.position, setpoint.velocity) }
        m_pivotMotor.setVoltage(output + feedForward!!)
    }

    override fun getMeasurement(): Double {
        val preConversion: Double = convert360To180(((canCoderPositionDegrees)) % 360) * Math.PI / 180
        return preConversion + 3.425 - Math.PI
    }

    private fun convert360To180(angle: Double): Double {
        return (angle + 180) % 360 - 180
    }

    private val canCoderPositionDegrees: Double
        get() = ((m_canCoder.absolutePosition.valueAsDouble * 360 + 360) - 63) % 360

    override fun periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(measurement), m_controller.setpoint)
        }
    }

    fun goHome() {
        setGoal(Position.HOME.rotations)
    }

    fun goToDangle() {
        setGoal(Position.DANGLE.rotations)
    }

    fun goToAmpFirePosition() {
        setGoal(Position.AMP_SHOOT.rotations)
    }

    fun stopArm() {
        m_pivotMotor.stopMotor()
    }

    val inAmpFirePosition
        get() = abs(measurement - Position.AMP_SHOOT.rotations) <= Constants.AmpArm.PIVOT_TOLERANCE * 5

    val inDanglePosition
        get() = abs(measurement - Position.DANGLE.rotations) <= Constants.AmpArm.PIVOT_TOLERANCE * 5

    fun armInPosition(pos: Position): Boolean {
        return abs(measurement - pos.rotations) <= Constants.AmpArm.PIVOT_TOLERANCE * 5
    }
}
