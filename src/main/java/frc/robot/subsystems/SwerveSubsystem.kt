package frc.robot.subsystems

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.util.Limelight
import frc.lib.util.OdometryImpl
import frc.lib.util.math.Conversions
import frc.lib.util.math.Conversions.rotationsToMeters
import frc.lib.util.math.MathConstants
import frc.robot.CTREConfigs
import frc.robot.Constants
import frc.robot.Constants.PoseConfig
import frc.robot.RobotContainer
import frc.robot.commands.Autos
import frc.robot.utils.SwerveModuleConstants
import kotlin.math.atan
import kotlin.math.sqrt


class SwerveModule(val swerveModuleNumber: Int, private val constants: SwerveModuleConstants) {
    private val driveMotor: TalonFX
    private val angleMotor: TalonFX
    private val angleEncoder: CANcoder

    var currentState: SwerveModuleState = SwerveModuleState()
        private set
    private var desiredState: SwerveModuleState = SwerveModuleState()

    //    private val drivePIDController: PIDController = PIDController(0.1, 0.0, 0.0)
//    private val steeringPIDController: PIDController = PIDController(0.1, 0.0, 0.0)
    private val driveFeedForward =
        SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA)

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

//    fun getPosition(): SwerveModulePosition {
//        return SwerveModulePosition(
//            rotationsToMeters(driveMotor.position.value, Constants.Swerve.WHEEL_CIRCUMFERENCE),
//            Rotation2d.fromRotations(angleMotor.position.value)
//        )
//    }

    val position: SwerveModulePosition
        get() = SwerveModulePosition(
            rotationsToMeters(driveMotor.position.value, Constants.Swerve.WHEEL_CIRCUMFERENCE),
            Rotation2d.fromRotations(angleMotor.position.value)
        )

    fun resetToAbsolute() {
        val absolutePosition: Double =
            Rotation2d.fromRotations(angleEncoder.absolutePosition.value).rotations - constants.angleOffset.rotations
        angleMotor.setPosition(absolutePosition)
    }

    private var counter = 0
    fun periodic() {
        currentState = SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.velocity.value, Constants.Swerve.WHEEL_CIRCUMFERENCE),
            Rotation2d.fromRotations(angleMotor.position.value)
        )

        counter = (counter + 1) % 1000

        if (counter % 20 == 0) { // every 20 cycles
            SmartDashboard.putNumber(
                "Speed",
                Conversions.RPSToMPS(driveMotor.velocity.value, Constants.Swerve.WHEEL_CIRCUMFERENCE)
            )
            SmartDashboard.putNumber("Angle", Rotation2d.fromRotations(angleMotor.position.value).degrees)
        }
    }
}


object SwerveSubsystem : SubsystemBase() {
    var poseEstimator: SwerveDrivePoseEstimator? = null
    val odometryImpl: OdometryImpl

    var limelightShooter: Limelight
    var limelightArm: Limelight
    private var gyro: Pigeon2 = Pigeon2(Constants.Swerve.PIGEON_ID, Constants.PIGEON_CAN_NAME)

    private val field = Field2d()


    val alignPID: PIDController = PIDController(
        Constants.Swerve.AUTO_ALIGN_KP,
        Constants.Swerve.AUTO_ALIGN_KI.toDouble(),
        Constants.Swerve.AUTO_ALIGN_KD.toDouble()
    )

    //    private var speedMultiplier = 1.0
    private var speedMultiplier = .2
    private val m_swerveModules: Array<SwerveModule> = arrayOf(
        SwerveModule(0, Constants.Modules.FrontLeft.constants),
        SwerveModule(1, Constants.Modules.FrontRight.constants),
        SwerveModule(2, Constants.Modules.BackLeft.constants),
        SwerveModule(3, Constants.Modules.BackRight.constants)
    )

    init {
        gyro.configurator.apply(Pigeon2Configuration())
        gyro.setYaw(0.0)

        odometryImpl = OdometryImpl()

        limelightShooter = Limelight(Constants.LimelightConstants.LIMELIGHT_SHOOTER)
        limelightArm = Limelight(Constants.LimelightConstants.LIMELIGHT_ARM)

        limelightShooter.pipeline = Constants.LimelightConstants.LIMELIGHT_SHOOTER_TAG_PIPELINE
        limelightArm.pipeline = Constants.LimelightConstants.LIMELIGHT_ARM_TAG_PIPELINE

        alignPID.enableContinuousInput(0.0, 360.0)
        alignPID.setTolerance(1.0)

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::robotRelativeSpeeds,
            this::driveRobotRelative,
            HolonomicPathFollowerConfig(
                PIDConstants(
                    Constants.AutonConstants.TRANSLATION_KP,
                    Constants.AutonConstants.TRANSLATION_KI,
                    Constants.AutonConstants.TRANSLATION_KD,
                ),
                PIDConstants(
                    Constants.AutonConstants.ROTATION_KP,
                    Constants.AutonConstants.ROTATION_KI,
                    Constants.AutonConstants.ROTATION_KD
                ),
                4.3,
                Constants.Swerve.TRACK_WIDTH / sqrt(2.0),
                ReplanningConfig()
            ),
            {
                val alliance = DriverStation.getAlliance()
                if (alliance.isPresent) {
                    return@configureHolonomic alliance.get() == DriverStation.Alliance.Red
                }
                return@configureHolonomic false
            },
            this
        )
    }

    private fun resetModulesToAbsolute() {
        m_swerveModules.forEach { it.resetToAbsolute() }
//        for (mod in m_swerveModules) {
//            mod.resetToAbsolute()
//        }
    }

    fun initializePoseEstimator() {
        DriverStation.reportWarning("Initializing pose estimator", false)
        val origin: Pose2d?

        if (RobotContainer.alliance == DriverStation.Alliance.Blue) {
            origin = Constants.BlueTeamPoses.BLUE_ORIGIN
        } else {
            origin = Constants.RedTeamPoses.RED_ORIGIN
        }

        resetModulesToAbsolute()

        poseEstimator = SwerveDrivePoseEstimator(
            Constants.Swerve.SWERVE_KINEMATICS,
            gyroYaw,
            modulePositions,
            origin,
            odometryImpl.createStdDevs(
                PoseConfig.K_POSITION_STD_DEV_X,
                PoseConfig.K_POSITION_STD_DEV_Y,
                PoseConfig.K_POSITION_STD_DEV_THETA
            ),
            odometryImpl.createStdDevs(
                PoseConfig.K_VISION_STD_DEV_X,
                PoseConfig.K_VISION_STD_DEV_Y,
                PoseConfig.K_VISION_STD_DEV_THETA
            )
        )
    }

    fun toggleSpeedChange() {
        speedMultiplier = if (speedMultiplier == 1.0) 0.2 else 1.0
    }

    fun calculateTurnAngle(target: Pose2d, robotAngle: Double): Double {
        val tx = target.x
        val ty = target.y
        val rx = getRelativePose().x
        val ry = getRelativePose().y

        val requestedAngle = atan((ty - ry) / (tx - rx)) * (180 / Math.PI)
        val calculatedAngle = (180 - robotAngle + requestedAngle)

        return ((calculatedAngle + 360) % 360)
    }


    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean, isOpenLoop: Boolean) {
        val newStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
            if (fieldRelative) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.x * speedMultiplier,
                    translation.y * speedMultiplier,
                    rotation * speedMultiplier,
                    heading
                )
            } else {
                ChassisSpeeds(
                    translation.x * speedMultiplier,
                    translation.y * speedMultiplier,
                    rotation * speedMultiplier
                )
            }

        )

        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.Swerve.MAX_SPEED)

        for (module in m_swerveModules) {
            module.setDesiredState(newStates[module.swerveModuleNumber], isOpenLoop)
        }
    }

    fun getPose(): Pose2d {
        if (poseEstimator == null) return Pose2d()
        return poseEstimator!!.estimatedPosition
    }


    fun getRelativePose(): Pose2d {
        if (poseEstimator == null) return Pose2d()

        return if (RobotContainer.alliance == DriverStation.Alliance.Blue) {
            poseEstimator!!.estimatedPosition
        } else {
            poseEstimator!!.estimatedPosition.relativeTo(Constants.RedTeamPoses.RED_ORIGIN)
        }
    }

    private fun setPose(pose: Pose2d?) {
        poseEstimator?.resetPosition(gyroYaw, modulePositions, pose)
    }

    fun setHeading(heading: Rotation2d?) {
        poseEstimator?.resetPosition(gyroYaw, modulePositions, Pose2d(getPose().translation, heading))
    }

    fun zeroHeading() {
        val zeroPose = if (RobotContainer.alliance == DriverStation.Alliance.Blue) {
            Pose2d(getPose().translation, Rotation2d())
        } else {
            Pose2d(poseEstimator!!.estimatedPosition.translation, Rotation2d.fromDegrees(180.0))
        }
        poseEstimator?.resetPosition(gyroYaw, modulePositions, zeroPose)
    }

    private fun driveRobotRelative(chassisSpeeds: ChassisSpeeds?) {
        val swerveModuleStates: Array<SwerveModuleState> =
            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds)

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED)

        for (mod in m_swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.swerveModuleNumber], false)
        }
    }

    private fun addVisionToPose(limelight: Limelight) {
        if (poseEstimator == null) return

        val visionMeasurement = odometryImpl.getVisionMeasurement(limelight)
        if (visionMeasurement != null) {
            poseEstimator?.addVisionMeasurement(visionMeasurement, limelight.limelightLatency)
        }
    }

    val isLowGear
        get() = speedMultiplier == 0.2

    val heading: Rotation2d
        get() = getRelativePose().rotation

    private val gyroYaw: Rotation2d
        get() = Rotation2d.fromDegrees(gyro.yaw.value)

    private val modulePositions: Array<SwerveModulePosition>
        get() {
            val positions = Array(4) { SwerveModulePosition(0.0, Rotation2d(0.0)) }
            for (mod in m_swerveModules) {
                positions[mod.swerveModuleNumber] = mod.position
            }
            return positions
        }

    private val moduleStates: Array<SwerveModuleState>
        get() {
            val states = Array(m_swerveModules.size) { SwerveModuleState() }
            for (mod in m_swerveModules) {
                states[mod.swerveModuleNumber] = mod.currentState
            }
            return states
        }

    private val robotRelativeSpeeds: ChassisSpeeds
        get() {
            return Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(*moduleStates) // use * spread operator to spread the array of swerve states
        }


    private var counter = 0
    override fun periodic() {
        // Update pose estimator if it exists
        if (poseEstimator != null) {
            poseEstimator?.update(gyroYaw, modulePositions)
        }

        // Only update odometry periodically (e.g., every 5 cycles)
        if (counter % 5 == 0) {
            odometryImpl.periodic()
        }

        // Update each swerve module (consider reducing frequency if needed)
        if (counter % 4 == 0) { // every two cycles
            for (module in m_swerveModules) {
                module.periodic()
            }
        }

        if (useVision) {
            addVisionToPose(limelightShooter)
            addVisionToPose(limelightArm)
        }

        // Update field display on SmartDashboard periodically (e.g., every 10 cycles)
        if (counter % 10 == 0) {
            field.robotPose = getPose()
            SmartDashboard.putData("Real Field", field)
        }

        // Log to the logger periodically (e.g., every 10 cycles)
//        if (counter % 10 == 0) {
//            Logger.recordOutput("odometry/robot", getPose())
//            Logger.recordOutput("odometry/robot3d", Pose3d(getPose()))
//        }

        // Increment the counter (reset to avoid overflow)
        counter = (counter + 1) % 1000
    }

    private val useVision: Boolean
        get() = (RobotContainer.robotState != RobotContainer.RobotState.AUTON || Autos.selectedAutonomousUseVision) && RobotContainer.useVision

    fun stop() {
        drive(MathConstants.TRANSLATION2D_ZERO, MathConstants.ROTATION_ZERO, fieldRelative = true, isOpenLoop = true)
    }
}
