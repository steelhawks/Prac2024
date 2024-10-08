package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants
import frc.robot.NoteStatus


object IntakeSubsystem : SubsystemBase() {
    private val m_intakeMotor1 = TalonFX(Constants.Intake.INTAKE_MOTOR_1_ID, Constants.CANIVORE_NAME)
    private val m_intakeMotor2 = TalonFX(Constants.Intake.INTAKE_MOTOR_2_ID, Constants.CANIVORE_NAME)
    private val m_forkMotor = TalonFX(Constants.Intake.FORK_MOTOR_ID, Constants.CANIVORE_NAME)

    private val intakeBeam = DigitalInput(Constants.Intake.BEAM_BREAKER_INTAKE)
    private val armBeam = DigitalInput(Constants.Intake.BEAM_BREAKER_ARM)

    var noteStatus: NoteStatus = NoteStatus.NOTHING
    var intakeToArmInterrupted: Boolean = false

    enum class IntakeDirection {
        TO_SHOOTER,
        TO_AMP_ARM,
        TO_INTAKE
    }

    init {
        configureMotors()
    }

    fun fork(dir: IntakeDirection) {
        when (dir) {
            IntakeDirection.TO_SHOOTER -> {
                m_intakeMotor1.set(-Constants.Intake.INTAKE_FEED_SPEED)
                m_intakeMotor2.set(-Constants.Intake.INTAKE_FEED_SPEED)
                m_forkMotor.set(Constants.Intake.FORK_SPEED)
            }
            IntakeDirection.TO_AMP_ARM -> {
                m_intakeMotor1.set(-Constants.Intake.INTAKE_FEED_SPEED / 3)
                m_intakeMotor2.set(-Constants.Intake.INTAKE_FEED_SPEED / 3)
                m_forkMotor.set(-Constants.Intake.FORK_SPEED / 3)
            }
            IntakeDirection.TO_INTAKE -> {
                m_intakeMotor1.set(-Constants.Intake.INTAKE_FEED_SPEED / 3.5)
                m_intakeMotor2.set(-Constants.Intake.INTAKE_FEED_SPEED / 3.5)
                m_forkMotor.set(Constants.Intake.FORK_SPEED)
            }
        }
    }

    fun intake() {
        m_forkMotor.setVoltage(1.0)

        m_intakeMotor1.set(-Constants.Intake.INTAKE_SPEED)
        m_intakeMotor2.set(-Constants.Intake.INTAKE_SPEED)
    }

    fun intakeReverse() {
        m_intakeMotor1.set(Constants.Intake.INTAKE_SPEED)
        m_intakeMotor2.set(Constants.Intake.INTAKE_SPEED)
    }

    fun intakeToArm() {
        m_forkMotor.setVoltage(-0.8)

        m_intakeMotor1.set(-Constants.Intake.FORK_SPEED / 2)
        m_intakeMotor2.set(-Constants.Intake.FORK_SPEED / 2)
    }

    private fun configureMotors() {
        m_intakeMotor1.inverted = false
        m_intakeMotor2.inverted = true
        m_forkMotor.inverted = true

        m_intakeMotor1.setNeutralMode(NeutralModeValue.Coast)
        m_intakeMotor2.setNeutralMode(NeutralModeValue.Coast)
        m_forkMotor.setNeutralMode(NeutralModeValue.Coast)
    }

    fun intakeLEDCommand(alliance: DriverStation.Alliance): Command {
        return LEDSubsystem.flashCommand(
            if (alliance == DriverStation.Alliance.Red) LEDSubsystem.LEDColor.RED else if (alliance == DriverStation.Alliance.Blue) LEDSubsystem.LEDColor.BLUE else LEDSubsystem.LEDColor.GREEN,
            0.1,
            1.0
        )
    }

    fun stop() {
        m_intakeMotor1.stopMotor()
        m_intakeMotor2.stopMotor()
        m_forkMotor.stopMotor()
    }

    val intakeBeamBroken get() = !intakeBeam.get()
    val armBeamBroken get() = !armBeam.get()

    override fun periodic() {
        SmartDashboard.putBoolean("intake/intake beam", intakeBeamBroken)
        SmartDashboard.putBoolean("intake/arm beam", armBeamBroken)
    }
}