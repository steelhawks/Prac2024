package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.SwerveModuleConstants
import com.ctre.phoenix6.hardware.TalonFX
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
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Translation2d
import frc.lib.util.math.MathConstants
import frc.robot.CTREConfigs

class SwerveModule(val swerveModuleNumber: Int, private val constants: SwerveModuleConstants) {
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
        angleMotor = TalonFX(constants.angleMotorID, Constants.CANIVORE_NAME)
        angleMotor.configurator.apply(CTREConfigs.swerveAngleFXConfig)

        // drive motor config
        driveMotor = TalonFX(constants.driveMotorID, Constants.CANIVORE_NAME)
        driveMotor.configurator.apply(CTREConfigs.swerveAngleFXConfig)
        driveMotor.configurator.setPosition(0.0)

        driveMotor.setNeutralMode(NeutralModeValue.Brake)
        angleMotor.setNeutralMode(NeutralModeValue.Brake)
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

    // max fong wuz here

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

    fun resetToAbsolute() {
        val absolutePosition: Double = Rotation2d.fromRotations(angleEncoder.absolutePosition.value).rotations - constants.angleOffset.rotations
        angleMotor.setPosition(absolutePosition)
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
//    private var speedMultiplier = 1.0
    private var speedMultiplier = .1
    private val m_swerveModules: Array<SwerveModule> = arrayOf(
        SwerveModule(0, Constants.Modules.FrontLeft.constants),
        SwerveModule(1, Constants.Modules.FrontRight.constants),
        SwerveModule(2, Constants.Modules.BackLeft.constants),
        SwerveModule(3, Constants.Modules.BackRight.constants)
    )

    init {
        resetModulesToAbsolute()
    }

    private fun resetModulesToAbsolute() {
        for (mod in m_swerveModules) {
            mod.resetToAbsolute()
        }
    }

//    private val frontLeftModule = SwerveModule(Constants.Modules.FrontLeft.constants)
//    private val frontRightModule = SwerveModule(Constants.Modules.FrontRight.constants)
//    private val backLeftModule = SwerveModule(Constants.Modules.BackLeft.constants)
//    private val backRightModule = SwerveModule(Constants.Modules.BackRight.constants)

    fun toggleSpeedChange() {
//        speedMultiplier = if (speedMultiplier == 1.0) 0.2 else 1.0
        speedMultiplier = .1
    }


    fun drive(translation: Translation2d, rotation: Double, isOpenLoop: Boolean) {
        val newStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds(
                translation.x * speedMultiplier,
                translation.y * speedMultiplier,
                rotation * speedMultiplier
            )
        )

//        val newStates: Array<SwerveModuleState> = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(desiredSpeed)

        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.Swerve.MAX_SPEED)

        for (module in m_swerveModules) {
            module.setDesiredState(newStates[module.swerveModuleNumber], isOpenLoop)
        }

//        frontLeftModule.setDesiredState(newStates[0], isOpenLoop)
//        frontRightModule.setDesiredState(newStates[1], isOpenLoop)
//        backLeftModule.setDesiredState(newStates[2], isOpenLoop)
//        backRightModule.setDesiredState(newStates[3], isOpenLoop)


//        SmartDashboard.putNumberArray("Setting chassis speeds: ", doubleArrayOf(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, desiredSpeed.omegaRadiansPerSecond))
//        println("Setting chassis speeds: vx=${desiredSpeed.vxMetersPerSecond}, vy=${desiredSpeed.vyMetersPerSecond}, omega=${desiredSpeed.omegaRadiansPerSecond}")
    }

    override fun periodic() {
//        frontLeftModule.periodic()
//        frontRightModule.periodic()
//        backLeftModule.periodic()
//        backRightModule.periodic()
        for (module in m_swerveModules) {
            module.periodic()
        }

//        val simulatorLoggingState = doubleArrayOf(
//            frontLeftModule.desiredState.angle.degrees,
//            frontLeftModule.desiredState.speedMetersPerSecond,
//            frontRightModule.desiredState.angle.degrees,
//            frontRightModule.desiredState.speedMetersPerSecond,
//            backLeftModule.desiredState.angle.degrees,
//            backLeftModule.desiredState.speedMetersPerSecond,
//            backRightModule.desiredState.angle.degrees,
//            backRightModule.desiredState.speedMetersPerSecond,
//        )

        val simulatorLoggingState = doubleArrayOf(
            m_swerveModules[0].desiredState.angle.degrees,
            m_swerveModules[0].desiredState.speedMetersPerSecond,
            m_swerveModules[1].desiredState.angle.degrees,
            m_swerveModules[1].desiredState.speedMetersPerSecond,
            m_swerveModules[2].desiredState.angle.degrees,
            m_swerveModules[2].desiredState.speedMetersPerSecond,
            m_swerveModules[3].desiredState.angle.degrees,
            m_swerveModules[3].desiredState.speedMetersPerSecond
        )

//        val realRobotMotorLoggingState = doubleArrayOf(
//            frontLeftModule.currentState.angle.degrees,
//            frontLeftModule.currentState.speedMetersPerSecond,
//            frontRightModule.currentState.angle.degrees,
//            frontRightModule.currentState.speedMetersPerSecond,
//            backLeftModule.currentState.angle.degrees,
//            backLeftModule.currentState.speedMetersPerSecond,
//            backRightModule.currentState.angle.degrees,
//            backRightModule.currentState.speedMetersPerSecond
//        )

        val realRobotMotorLoggingState = doubleArrayOf(
            m_swerveModules[0].currentState.angle.degrees,
            m_swerveModules[0].currentState.speedMetersPerSecond,
            m_swerveModules[1].currentState.angle.degrees,
            m_swerveModules[1].currentState.speedMetersPerSecond,
            m_swerveModules[2].currentState.angle.degrees,
            m_swerveModules[2].currentState.speedMetersPerSecond,
            m_swerveModules[3].currentState.angle.degrees,
            m_swerveModules[3].currentState.speedMetersPerSecond,
        )

        SmartDashboard.putNumberArray("Simulator SwerveModule States", simulatorLoggingState)
        SmartDashboard.putNumberArray("Real Robot SwerveModule States", realRobotMotorLoggingState)
    }

    fun stop() {
        drive(MathConstants.TRANSLATION2D_ZERO, MathConstants.ROTATION_ZERO, false)
    }
}
