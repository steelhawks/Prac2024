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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import com.ctre.phoenix6.hardware.CANcoder
import frc.robot.CTREConfigs

class SwerveModule(private val constants: SwerveModuleConstants) {
    private val driveMotor: TalonFX
    private val angleMotor: TalonFX
    private val angleEncoder: CANcoder

    var currentState: SwerveModuleState = SwerveModuleState()
        private set
    var desiredState: SwerveModuleState = SwerveModuleState()
        private set

//    private val drivePIDController: PIDController = PIDController(0.1, 0.0, 0.0)
//    private val steeringPIDController: PIDController = PIDController(0.1, 0.0, 0.0)
    private val driveFeedForward = SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA)

    private val driveDutyCycle = DutyCycleOut(0.0)
    private val driveVelocity = VelocityVoltage(0.0)
    private val anglePosition = PositionVoltage(0.0)

    init {
        println("SwerveModule initialized")

        angleEncoder = CANcoder(constants.cancoderID, Constants.CANIVORE_NAME)
        angleEncoder.configurator.apply(CTREConfigs.swerveCANcoderConfig)

        // angle motor config
        angleMotor = TalonFX(constants.driveMotorID, Constants.CANIVORE_NAME)
        angleMotor.configurator.apply(CTREConfigs.swerveAngleFXConfig)

        // drive motor config
        driveMotor = TalonFX(constants.driveMotorID, Constants.CANIVORE_NAME)
        driveMotor.configurator.apply(CTREConfigs.swerveAngleFXConfig)
        driveMotor.configurator.setPosition(0.0)
    }

    fun setDesiredState(state: SwerveModuleState, isOpenLoop: Boolean) {
        desiredState = SwerveModuleState.optimize(state, currentState.angle)
        setAngle(desiredState.angle)
        setSpeed(desiredState.speedMetersPerSecond, isOpenLoop)
    }

    private fun setAngle(angle: Rotation2d) {
        angleMotor.setControl(anglePosition.withPosition(angle.rotations))
        SmartDashboard.putNumber("Angle degrees", angle.degrees)
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

        SmartDashboard.putNumber("Speed", speed)
    }

    fun periodic() {
        currentState = SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.velocity.value, Constants.Swerve.WHEEL_CIRCUMFERENCE),
            Rotation2d.fromRotations(angleMotor.position.value)
        )

        SmartDashboard.putNumber("Speed", Conversions.RPSToMPS(driveMotor.velocity.value, Constants.Swerve.WHEEL_CIRCUMFERENCE))
        SmartDashboard.putNumber("Angle", Rotation2d.fromRotations(angleMotor.position.value).degrees)
    }
}


object SwerveSubsystem : SubsystemBase() {
    var speedMultiplier = 1.0
        private set

    private val frontLeftModule = SwerveModule(Constants.Modules.FrontLeft.constants)
    private val frontRightModule = SwerveModule(Constants.Modules.FrontRight.constants)
    private val backLeftModule = SwerveModule(Constants.Modules.BackLeft.constants)
    private val backRightModule = SwerveModule(Constants.Modules.BackRight.constants)

    fun toggleSpeedChange() {
        speedMultiplier = if (speedMultiplier == 1.0) 0.2 else 1.0
    }


    fun drive(desiredSpeed: ChassisSpeeds, isOpenLoop: Boolean) {
        val newStates: Array<SwerveModuleState> = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(desiredSpeed)

        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.Swerve.MAX_SPEED)

        frontLeftModule.setDesiredState(newStates[0], isOpenLoop)
        frontRightModule.setDesiredState(newStates[1], isOpenLoop)
        backLeftModule.setDesiredState(newStates[2], isOpenLoop)
        backRightModule.setDesiredState(newStates[3], isOpenLoop)


        SmartDashboard.putNumberArray("Setting chassis speeds: ", doubleArrayOf(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, desiredSpeed.omegaRadiansPerSecond))
//        println("Setting chassis speeds: vx=${desiredSpeed.vxMetersPerSecond}, vy=${desiredSpeed.vyMetersPerSecond}, omega=${desiredSpeed.omegaRadiansPerSecond}")
    }

    override fun periodic() {
        frontLeftModule.periodic()
        frontRightModule.periodic()
        backLeftModule.periodic()
        backRightModule.periodic()

        val loggingState = doubleArrayOf(
            frontLeftModule.desiredState.angle.degrees,
            frontLeftModule.desiredState.speedMetersPerSecond,
            frontRightModule.desiredState.angle.degrees,
            frontRightModule.desiredState.speedMetersPerSecond,
            backLeftModule.desiredState.angle.degrees,
            backLeftModule.desiredState.speedMetersPerSecond,
            backRightModule.desiredState.angle.degrees,
            backRightModule.desiredState.speedMetersPerSecond,
        )

        SmartDashboard.putNumberArray("SwerveMod States", loggingState)
    }

    fun stop() {
        drive(ChassisSpeeds(0.0, 0.0, 0.0), false)
    }
}
