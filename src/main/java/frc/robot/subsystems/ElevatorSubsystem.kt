package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants

object ElevatorSubsystem : ProfiledPIDSubsystem(
    ProfiledPIDController(
        Constants.Elevator.KP,
        Constants.Elevator.KI,
        Constants.Elevator.KD,
        TrapezoidProfile.Constraints(
            Constants.Elevator.K_MAX_VELOCITY_PER_SECOND,
            Constants.Elevator.K_MAX_ACCELERATION_PER_SEC_SQUARED
        )
    ),
    0.0
) {
    enum class ElevatorLevel(val rotations: Double) {
        HOME(Constants.Elevator.HOME_ROTATIONS),
        TRAP(Constants.Elevator.TRAP_ROTATIONS),
        AMP(Constants.Elevator.AMP_ROTATIONS),
        CLIMB(Constants.Elevator.CLIMB_ROTATIONS)
    }

    private val elevatorLeft = TalonFX(Constants.Elevator.LEFT_MOTOR_ID, Constants.CANIVORE_NAME)
    private val elevatorRight = TalonFX(Constants.Elevator.RIGHT_MOTOR_ID, Constants.CANIVORE_NAME)

    private val lowerLimitSwitch = DigitalInput(Constants.Elevator.LIMIT_SWITCH_LOWER_CHANNEL)
    private val canCoder = CANcoder(Constants.Elevator.CANCODER_ID, Constants.CANIVORE_NAME)
    private val initialValue = canCoder.position.value


    private val feedforward = ElevatorFeedforward(Constants.Elevator.KS, Constants.Elevator.KG, Constants.Elevator.KV)

    init {
        controller.setTolerance(Constants.Elevator.TOLERANCE)
        canCoder.configurator.apply(CANcoderConfiguration().withMagnetSensor(MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)))

        configureMotors()

        if (atElevatorMin) {
            resetCANCoder()
        } else {
            DriverStation.reportWarning("ELEVATOR IS NOT AT MINIMUM... DO NOT MOVE", false)
        }

        enable()
    }

    private fun configureMotors() {
        elevatorLeft.setNeutralMode(NeutralModeValue.Brake)
        elevatorRight.setNeutralMode(NeutralModeValue.Brake)

        elevatorLeft.inverted = false
        elevatorRight.inverted = true
    }

    // manual control by straight calling this doesnt work because of the pid
    // to make this work make a command that disables the pid controller then uses this.
    fun controlElevator(isDown: Boolean) {
        println("Running control elevator")
        var speed: Double = Constants.Elevator.MANUAL_ELEVATOR_SPEED
        if (isDown) {
            if (atElevatorMin) {
                stopElevator()
                return
            }
        } else {
            if (atElevatorMax) {
                stopElevator()
                return
            }
            speed = -speed
        }
        elevatorLeft.set(speed)
        elevatorRight.set(speed)
    }

    fun getHomeCommand(): Command {
        return Commands.sequence(
            Commands.runOnce(this::disable)
                .andThen(
                    Commands.run({
                        controlElevator(true)
                    }, this).until(this::atElevatorMin)
                ).andThen(Commands.runOnce(this::stopElevator))
        )
    }

    fun goHome() {
        setGoal(ElevatorLevel.HOME.rotations)
    }

    fun stopElevator() {
        elevatorLeft.stopMotor()
        elevatorRight.stopMotor()
    }

    private fun resetCANCoder() {
        canCoder.setPosition(0.0)
    }

    fun elevatorInPosition(pos: ElevatorLevel): Boolean {
        return Math.abs(measurement - pos.rotations) <= Constants.Elevator.TOLERANCE * 5
    }

    /** Runs LED Command when elevator is at minimum or maximum when [atElevatorMin] or [atElevatorMax] is true */
    fun elevatorLEDCommand(): Command {
        return LEDSubsystem.flashCommand(
            LEDSubsystem.LEDColor.PINK,
            .2,
            2.0
        )
    }

    val atElevatorMax: Boolean
        get() = measurement >= Constants.Elevator.MAX_ROTATIONS

    val atElevatorMin: Boolean
        get() = !lowerLimitSwitch.get()

    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State?) {
        setpoint?.velocity?.let { SmartDashboard.putNumber("elevator/Setpoint Velocity", it) }
        setpoint?.position?.let { SmartDashboard.putNumber("elevator/Setpoint Position", it) }
        SmartDashboard.putNumber("elevator/Output", output)

        val feedforward = setpoint?.let { feedforward.calculate(it.velocity) }
        val volts = -(output + feedforward!!)

        if (atElevatorMin && volts >= 0.0 || atElevatorMax && volts <= 0.0)  {
            stopElevator()
            return
        }

        elevatorLeft.setVoltage(volts)
        elevatorRight.setVoltage(volts)
    }

    override fun getMeasurement(): Double = canCoder.position.value

    fun getAmpCommand(): Command {
        return InstantCommand({
            setGoal(ElevatorLevel.AMP.rotations)
            enable()
        })
    }

    override fun periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(measurement), m_controller.setpoint)
        }

        SmartDashboard.putBoolean("elevator/lower limit hit", atElevatorMin)
        SmartDashboard.putNumber("elevator/rotations", measurement)

        SmartDashboard.putBoolean("elevator/at position", elevatorInPosition(ElevatorLevel.AMP))
    }
}
