import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.SwerveModuleConstants
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants

class SwerveModule(constants: SwerveModuleConstants) {
    private val driveMotor: TalonFX = TalonFX(constants.driveMotorID)
    private val steeringMotor: TalonFX = TalonFX(constants.angleMotorID)
    var currentState: SwerveModuleState
        private set
    lateinit var desiredState: SwerveModuleState
        private set

    private val drivePIDController: PIDController
    private val steeringPIDController: PIDController

    // simulator
//    private val steeringSimulation = FlywheelSim(
//        DCMotor.getFalcon500(1),
//        150.0 / 7.0,
//        0.004
//    )

    fun setDesiredState(state: SwerveModuleState) {
        currentState = state
    }

    init {
        println("SwerveModule")
        currentState = SwerveModuleState()

        drivePIDController = PIDController(0.1, 0.0, 0.0)
        steeringPIDController = PIDController(0.1, 0.0, 0.0)
    }

    fun periodic() {
//        steeringSimulation.update(0.02)
//
//        // get new simulated angle change
//        val simulatedAngleDiffRad = steeringSimulation.angularVelocityRadPerSec * 0.02
//
//        // update curr state
//        currentState = SwerveModuleState(
//            currentState.speedMetersPerSecond,
//            Rotation2d.fromDegrees(currentState.angle.degrees + Units.radiansToDegrees(simulatedAngleDiffRad))
//        )
        // set new angle
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

        frontLeftModule.setDesiredState(newStates[0])
        frontRightModule.setDesiredState(newStates[1])
        backLeftModule.setDesiredState(newStates[2])
        backRightModule.setDesiredState(newStates[3])
    }

    override fun periodic() {
        // front left, front right, back left, back right
        val loggingState = doubleArrayOf(
            frontLeftModule.currentState.angle.degrees, // mod 1 angle
            frontLeftModule.currentState.speedMetersPerSecond, // mod 1 speed m/s
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