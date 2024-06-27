package frc.robot.subsystems

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase;

object SwerveSubsystem : SubsystemBase() {
    lateinit var poseEstimator: SwerveDrivePoseEstimator

    init {

    }

    fun initPoseEstimator() {
        DriverStation.reportWarning("Initializing poseEstimator", false)
        val origin = Pose2d()
    }
}