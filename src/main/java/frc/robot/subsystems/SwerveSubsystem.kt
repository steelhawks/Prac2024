package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.SwerveModuleConstants
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.lib.util.math.Conversions
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.controller.SimpleMotorFeedforward

class SwerveModule(private val constants: SwerveModuleConstants) {
    private val driveMotor: TalonFX = TalonFX(constants.driveMotorID)
    private val steeringMotor: TalonFX = TalonFX(constants.angleMotorID)
    var currentState: SwerveModuleState = SwerveModuleState()
        private set
    var desiredState: SwerveModuleState = SwerveModuleState()
        private set

    private val drivePIDController: PIDController = PIDController(0.1, 0.0, 0.0)
    private val steeringPIDController: PIDController = PIDController(0.1, 0.0, 0.0)
    private val driveFeedForward = SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA)

    private val driveDutyCycle = DutyCycleOut(0.0)
    private val driveVelocity = VelocityVoltage(0.0)
    private val anglePosition = PositionVoltage(0.0)

    init {
        println("SwerveModule initialized")
    }

    fun setDesiredState(state: SwerveModuleState, isOpenLoop: Boolean) {
        desiredState = SwerveModuleState.optimize(state, currentState.angle)
        setAngle(desiredState.angle)
        setSpeed(desiredState.speedMetersPerSecond, isOpenLoop)
    }

    private fun setAngle(angle: Rotation2d) {
        anglePosition.withPosition(angle.rotations)
        steeringMotor.setControl(anglePosition)
    }

    private fun setSpeed(speed: Double, isOpenLoop: Boolean) {
        if (isOpenLoop) {
            driveDutyCycle.Output = speed / Constants.Swerve.MAX_SPEED
            driveMotor.setControl(driveDutyCycle)
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(speed, Constants.Swerve.WHEEL_CIRCUMFERENCE)
            driveVelocity.FeedForward = driveFeedForward.calculate(speed)
            driveMotor.setControl(driveVelocity)
        }
    }

    fun periodic() {
        currentState = SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.velocity.value, Constants.Swerve.WHEEL_CIRCUMFERENCE),
            Rotation2d.fromRotations(steeringMotor.position.value)
        )
    }
}

object SwerveSubsystem : SubsystemBase() {
    private val frontLeftModule = SwerveModule(Constants.Modules.FrontLeft.constants)
    private val frontRightModule = SwerveModule(Constants.Modules.FrontRight.constants)
    private val backLeftModule = SwerveModule(Constants.Modules.BackLeft.constants)
    private val backRightModule = SwerveModule(Constants.Modules.BackRight.constants)

    private val kinematics = Constants.Swerve.SWERVE_KINEMATICS

    fun setChassisSpeed(desiredSpeed: ChassisSpeeds) {
        val newStates: Array<SwerveModuleState> = kinematics.toSwerveModuleStates(desiredSpeed)

        frontLeftModule.setDesiredState(newStates[0], false)
        frontRightModule.setDesiredState(newStates[1], false)
        backLeftModule.setDesiredState(newStates[2], false)
        backRightModule.setDesiredState(newStates[3], false)
    }

    override fun periodic() {
        frontLeftModule.periodic()
        frontRightModule.periodic()
        backLeftModule.periodic()
        backRightModule.periodic()


        val loggingState = doubleArrayOf(
            frontLeftModule.currentState.angle.degrees,
            frontLeftModule.currentState.speedMetersPerSecond,
            frontRightModule.currentState.angle.degrees,
            frontRightModule.currentState.speedMetersPerSecond,
            backLeftModule.currentState.angle.degrees,
            backLeftModule.currentState.speedMetersPerSecond,
            backRightModule.currentState.angle.degrees,
            backRightModule.currentState.speedMetersPerSecond,
        )

        SmartDashboard.putNumberArray("SwerveMod States", loggingState)
    }

    fun stop() {
        setChassisSpeed(ChassisSpeeds(0.0, 0.0, 0.0))
    }
}
