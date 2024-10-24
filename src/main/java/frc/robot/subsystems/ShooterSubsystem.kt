package frc.robot.subsystems

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants
import kotlin.math.abs
import kotlin.math.cos

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

    private val m_feederMotor: TalonFX = TalonFX(Constants.Shooter.FEEDER_MOTOR_ID, Constants.CANIVORE_NAME)

    private val m_canCoder = CANcoder(Constants.Shooter.CANCODER_ID, Constants.CANIVORE_NAME)

    var autoShootingEnabled = true

    val firing: Boolean
        get() {
            val threshold = 10.0

            val bottomMotorVelocitySignal: StatusSignal<Double> = m_bottomShooterMotor.velocity
            val topMotorVelocitySignal: StatusSignal<Double> = m_topShooterMotor.velocity
            val bottomMotorVelocity: Double? = bottomMotorVelocitySignal.value
            val topMotorVelocity: Double? = topMotorVelocitySignal.value

            return (bottomMotorVelocity != null && bottomMotorVelocity > threshold) ||
                    (topMotorVelocity != null && topMotorVelocity > threshold)
        }

    private val canCoderVal
        get() = m_canCoder.absolutePosition.valueAsDouble * 360 - 24.7

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

    private var trigTargetAngle: Double = 0.0

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
        m_bottomShooterMotor.setNeutralMode(NeutralModeValue.Coast)
        m_topShooterMotor.setNeutralMode(NeutralModeValue.Coast)
        m_feederMotor.setNeutralMode(NeutralModeValue.Coast)

        m_pivotMotor.inverted = false
        m_pivotMotor.setPosition(0.0)
    }

    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State?) {
//        val feedforward = setpoint?.let { pivotFeedForward.calculate(it.position, setpoint.velocity) }
        val feedforward = pivotFeedForward.calculate(setpoint!!.position, setpoint.velocity)
        m_pivotMotor.setVoltage(output + feedforward)

        SmartDashboard.putNumber("shooter/feedforward", feedforward)
        SmartDashboard.putNumber("shooter/setpoint position", setpoint.velocity)
        SmartDashboard.putNumber("shooter/setpoint velocity", setpoint.velocity)
//        setpoint?.let {
//            val feedForward = pivotFeedForward.calculate(it.position, it.velocity)
//            m_pivotMotor.setVoltage(output + feedForward)
//            SmartDashboard.putNumber("shooter/feedforward", feedForward)
//            SmartDashboard.putNumber("shooter/setpoint position", it.position)
//            SmartDashboard.putNumber("shooter/setpoint velocity", it.velocity)
//        }
    }

    override fun getMeasurement(): Double {
        return (canCoderVal * Math.PI / 180) + 3.05
    }

    /** Feeder */
    fun feedToShooter() {
        m_feederMotor.set(10.0)
    }

    fun feedBackToIntake() {
        m_feederMotor.set(-0.8)
    }

    fun stopFeed() {
        m_feederMotor.stopMotor()
    }

    /** Shooter */
    fun goHome() { // test dis
//        if (!(abs(measurement - Constants.Shooter.HOME_POSITION) <= Constants.Shooter.PIVOT_TOLERANCE)) {
//            setGoal(Constants.Shooter.HOME_POSITION)
//        }
        setGoal(Constants.Shooter.HOME_POSITION)
    }

    fun stopShooter() {
        m_topShooterMotor.stopMotor()
        m_bottomShooterMotor.stopMotor()
    }

    fun stopPivot() {
        m_topShooterMotor.stopMotor()
    }

    fun manualShot() {
        m_topShooterMotor.set(Constants.Shooter.TOP_SHOOTER_SPEED)
        m_bottomShooterMotor.set(Constants.Shooter.BOTTOM_SHOOTER_SPEED)
    }

    fun rampShooter(topRPM: Double, bottomRPM: Double) {
        val topFeedforward = topShooterFeedForward.calculate(topRPM)
        val bottomFeedforward = bottomShooterFeedForward.calculate(bottomRPM)

        val topPID = topShooterPIDController.calculate(shooterTopRPM, topRPM)
        val bottomPID = bottomShooterPIDController.calculate(shooterBottomRPM, bottomRPM)

        m_topShooterMotor.setVoltage(topPID + topFeedforward)
        m_bottomShooterMotor.setVoltage(bottomPID + bottomFeedforward)
    }

    fun reverseToIntake() {
        m_topShooterMotor.set(-Constants.Shooter.TOP_SHOOTER_SPEED / 3)
        m_bottomShooterMotor.set(-Constants.Shooter.BOTTOM_SHOOTER_SPEED / 3)
    }

    fun controlShooter(isPivotingDown: Boolean?) {
        var voltage = cos(measurement) * Constants.Shooter.PIVOT_KG

        if (isPivotingDown == null) {
            m_pivotMotor.setVoltage(voltage)
            return
        }

        val adjustment = Constants.Shooter.PIVOT_KS + Constants.Shooter.MANUAL_PIVOT_VOLTAGE
        voltage += if (isPivotingDown) -adjustment else adjustment
        m_pivotMotor.setVoltage(voltage)
    }

    /** Runs LED Command when shooter is ready to shoot when [shooterBottomRPM], [shooterTopRPM], and [pivotAtSetpoint] are true */
    fun shooterLEDCommand(): Command {
        return LEDSubsystem.flashCommand(
            LEDSubsystem.LEDColor.PURPLE,
            0.1,
            2.0
        )
    }

    private val shooterTopRPM: Double
        get() = m_topShooterMotor.velocity.value * 60

    private val shooterBottomRPM: Double
        get() = m_bottomShooterMotor.velocity.value * 60

    // quick and dirty fix for subwoofer shot not setting to true
    // to fix maybe add a wait timer for it to actually get to this position
    // or maybe the time it takes for the shooters rpms to reach its time is enough time
    private val pivotAtSetpoint: Boolean
        get() = if (controller.goal.position == Constants.Shooter.HOME_POSITION) true else abs(measurement - controller.goal.position) <= Constants.Shooter.PIVOT_TOLERANCE * 1.5

    private val topShooterAtSetpoint: Boolean
        get() = abs((shooterTopRPM - topShooterPIDController.setpoint)) <= Constants.Shooter.SHOOTER_TOLERANCE * 1.5

    private val bottomShooterAtSetpoint: Boolean
        get() = abs((shooterBottomRPM - bottomShooterPIDController.setpoint)) <= Constants.Shooter.SHOOTER_TOLERANCE * 1.5

//    fun isShooterWithinRPMPercentageSetpoint(percentage: Double): Boolean {
//        return abs(shooterTopRPM / topShooterPIDController.setpoint) >= percentage && abs(shooterBottomRPM / bottomShooterPIDController.setpoint) >= percentage
//    }

    val isReadyToShoot: Boolean
        get() = topShooterAtSetpoint && bottomShooterAtSetpoint && pivotAtSetpoint


    private var counter = 0;
    override fun periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(measurement), m_controller.setpoint)
        }

        if (DriverStation.isDisabled()) {
            goHome()
        }

        counter = (counter + 1) % 1000 // reset to avoid overflow

        if (counter % 20 == 0) { // runs this every 20 cycles
            if (firing || isReadyToShoot) {
                SmartDashboard.putNumber("shooter/goal", controller.goal.position)
                SmartDashboard.putNumber("shooter/pivot angle", measurement)
                SmartDashboard.putNumber("shooter/pivot voltage", m_pivotMotor.motorVoltage.valueAsDouble)
            }

            SmartDashboard.putBoolean("shooter/is moving", firing)
            SmartDashboard.putBoolean("shooter/is ready to shoot", isReadyToShoot)
            SmartDashboard.putBoolean("shooter/at pivot setpoint", pivotAtSetpoint)
            SmartDashboard.putBoolean("shooter/top shooter at setpoint", topShooterAtSetpoint)
            SmartDashboard.putBoolean("shooter/bottom shooter at setpoint", bottomShooterAtSetpoint)
        }
    }
}
