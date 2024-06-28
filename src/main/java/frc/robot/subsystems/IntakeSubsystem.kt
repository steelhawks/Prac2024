package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


object IntakeSubsystem : SubsystemBase() {
    private val m_intakeMotor1 = TalonFX(Constants.Intake.INTAKE_MOTOR_1_ID, Constants.CANIVORE_NAME)
    private val m_intakeMotor2 = TalonFX(Constants.Intake.INTAKE_MOTOR_2_ID, Constants.CANIVORE_NAME)
    private val m_forkMotor = TalonFX(Constants.Intake.FORK_MOTOR_ID, Constants.CANIVORE_NAME)

    private val intakeBeam = DigitalInput(Constants.Intake.BEAM_BREAKER_INTAKE)
    private val armBeam = DigitalInput(Constants.Intake.BEAM_BREAKER_INTAKE)

    private val prevBeamBroken = false

    init {
        configureMotors()
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

    fun configureMotors() {
        m_intakeMotor1.inverted = false
        m_intakeMotor2.inverted = true
        m_forkMotor.inverted = true

        m_intakeMotor1.setNeutralMode(NeutralModeValue.Coast)
        m_intakeMotor2.setNeutralMode(NeutralModeValue.Coast)
        m_forkMotor.setNeutralMode(NeutralModeValue.Coast)
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