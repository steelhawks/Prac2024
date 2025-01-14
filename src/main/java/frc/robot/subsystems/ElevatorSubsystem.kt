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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.Constants.OperatorConstants
import frc.robot.RobotContainer
import frc.robot.RobotContainer.ManualMode
import frc.robot.commands.elevator.ManualElevatorControlCommand
import kotlin.math.abs
import kotlin.math.sign

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

//        enable()
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
        if ((isDown && atElevatorMin) || (!isDown && atElevatorMax)) {
            stopElevator()
            return
        }

        val speed = if (isDown) Constants.Elevator.MANUAL_ELEVATOR_SPEED
        else -Constants.Elevator.MANUAL_ELEVATOR_SPEED

        elevatorLeft.set(speed)
        elevatorRight.set(speed)
    }

    fun getHomeCommand(): Command {
        return SequentialCommandGroup(
            Commands.runOnce({
                disable()
                ArmSubsystem.goToDangle()
            })
                .andThen(
                    Commands.run({
                        controlElevator(true)
                    }, this).until(this::atElevatorMin).withTimeout(5.0)
                ).andThen(Commands.runOnce({
                    stopElevator()
                    ArmSubsystem.goHome()
                }))
        )
    }

    fun getManualElevatorCommand(isNormalMode: Trigger, operatorController: CommandXboxController): Command {
        return ConditionalCommand(
            InstantCommand({
                RobotContainer.elevatorManual = ManualMode.UNLOCKED
                ElevatorSubsystem.defaultCommand =
                    ManualElevatorControlCommand {
                        if (isNormalMode.asBoolean || abs(operatorController.getRawAxis(OperatorConstants.OPERATOR_LEFT_STICK_AXIS)) < Constants.Deadbands.CLIMB_DEADBAND) {
                            null
                        } else {
                            operatorController.getRawAxis(OperatorConstants.OPERATOR_LEFT_STICK_AXIS).sign > 0
                        }
                    }
            }),
            SequentialCommandGroup(
                InstantCommand({
                    RobotContainer.elevatorManual = ManualMode.LOCKED
                    defaultCommand.cancel()
                    removeDefaultCommand()
                }),
                ConditionalCommand(
                    getHomeCommand()
                        .until(this::atElevatorMin)
                        .andThen(
                            this::stopElevator
                        ),
                    Commands.runOnce({ ArmSubsystem.goHome() })
                ) { !atElevatorMin }
            )
        ) { RobotContainer.elevatorManual == ManualMode.LOCKED }
    }

    fun getAmpCommand(): Command {
        return InstantCommand({
            setGoal(ElevatorLevel.AMP.rotations)
            enable()
        })
    }

    fun getClimbCommand(): Command {
        return InstantCommand({
            setGoal(ElevatorLevel.CLIMB.rotations)
            enable()
        })
    }

    fun stopElevator() {
        elevatorLeft.stopMotor()
        elevatorRight.stopMotor()
    }

    fun resetCANCoder() {
        canCoder.setPosition(0.0)
    }

    fun elevatorInPosition(pos: ElevatorLevel): Boolean {
        if (pos == ElevatorLevel.HOME && atElevatorMin) {
            return true
        }

        return abs(measurement - pos.rotations) <= Constants.Elevator.TOLERANCE * 5
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

    override fun periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(measurement), m_controller.setpoint)
        }

        SmartDashboard.putBoolean("elevator/at amp level", elevatorInPosition(ElevatorLevel.AMP))
        SmartDashboard.putBoolean("elevator/lower limit hit", atElevatorMin)
        SmartDashboard.putNumber("elevator/rotations", measurement)
    }
}
