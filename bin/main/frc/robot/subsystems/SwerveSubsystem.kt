import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.SwerveModuleConstants
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants

class SwerveModule(constants: SwerveModuleConstants) {
    private val driveMotor: TalonFX = TalonFX(constants.driveMotorID)
    private val steeringMotor: TalonFX = TalonFX(constants.angleMotorID)
    var currentState: SwerveModuleState = SwerveModuleState(1.0, Rotation2d(Units.degreesToRadians(30.0)))
        private set

    init {
        println("SwerveModule")
    }
}

object SwerveSubsystem : SubsystemBase() {
    private val frontLeftModule = SwerveModule(Constants.Modules.FrontLeft.constants)
    private val frontRightModule = SwerveModule(Constants.Modules.FrontRight.constants)
    private val backLeftModule = SwerveModule(Constants.Modules.BackLeft.constants)
    private val backRightModule = SwerveModule(Constants.Modules.BackRight.constants)

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
}