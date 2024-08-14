// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.lib.util

import edu.wpi.first.math.Vector
import edu.wpi.first.math.util.Units
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.RobotContainer
import frc.robot.subsystems.SwerveSubsystem
import kotlin.math.atan

class OdometryImpl
/** Creates a new OdometryImpl.  */
{
    fun getDistance(target: Pose2d): Double {
        return SwerveSubsystem.getRelativePose().translation.getDistance(target.translation)
    }

    fun getPivotAngle(alliance: DriverStation.Alliance): Double {
        val height = Constants.StructureConstants.SPEAKER_HEIGHT - Constants.StructureConstants.SHOOTER_ROBOT_HEIGHT

        val base = if (alliance == DriverStation.Alliance.Red) {
            getDistance(Constants.RedTeamPoses.RED_SPEAKER_POSE)
        } else {
            getDistance(Constants.BlueTeamPoses.BLUE_SPEAKER_POSE)
        }

        return if (base > 3) {
            atan(height / base) + (base - 2.25) * 0.02 + (base - 3) * 0.01
        } else if (base > 2.25) {
            atan(height / base) + (base - 2.25) * 0.02
        } else {
            atan(height / base)
        }

        // return Math.atan(height / base);
    }

    //This is assuming that the robot is directly facing the target object
    fun getTurnAngle(target: Pose2d, robotAngle: Double): Double {
        val tx: Double = target.getX()
        val ty: Double = target.getY()

        val rx: Double = SwerveSubsystem.getPose().x
        val ry: Double = SwerveSubsystem.getPose().y

        val requestedAngle = atan((ty - ry) / (tx - rx)) * (180 / Math.PI)
        val calculatedAngle = (180 - robotAngle + requestedAngle)

        return ((calculatedAngle + 180) % 360) - 180
    }

    fun getVisionPoseError(limelight: Limelight?): Double {
        if (limelight == null || SwerveSubsystem.poseEstimator == null) return (-1).toDouble()
        val predictedPose: Pose2d? = limelight.visionPredictedRobotPose
        if (predictedPose != null) {
            return Math.abs(
                SwerveSubsystem.poseEstimator!!.getEstimatedPosition().getTranslation()
                    .getDistance(predictedPose.getTranslation())
            )
        }
        return (-1).toDouble()
    }

    fun isValidVisionMeasurement(limelight: Limelight?): Boolean {
        if (limelight == null) return false
        val predictedPose: Pose2d? = limelight.visionPredictedRobotPose
        if (predictedPose != null && (predictedPose.getX() != 0.0 && predictedPose.getY() != 0.0)) {
            if ((limelight.tagArea > Constants.LimelightConstants.MIN_AREA_OF_TAG) || limelight.numberOfTagsInView > 1) {
                return true
            }
        }
        return false
    }

    fun getVisionMeasurementWithoutYaw(limelight: Limelight?): Pose2d? {
        if (limelight == null) return null
        //added variable for predicted pose instead of calling function directly
        val predictedPose: Pose2d? = limelight.visionPredictedRobotPose
        if (isValidVisionMeasurement(limelight) && predictedPose != null) {
            return Pose2d(predictedPose.translation, SwerveSubsystem.heading)
        }
        return null
    }


    fun getVisionMeasurement(limelight: Limelight?): Pose2d? {
        if (limelight == null) return null
        val predictedPose: Pose2d? = limelight.visionPredictedRobotPose
        if (isValidVisionMeasurement(limelight) && predictedPose != null) {
            return predictedPose
        }
        return null
    }

    // Angle offset should be passed in as degrees
    fun createStdDevs(n1: Double, n2: Double, angleOffset: Double): Vector<N3> {
        return VecBuilder.fill(n1, n2, Units.degreesToRadians(angleOffset))
    }

    // Returns standard deviations using tag area; there MUST be multiple tags in sight
    private fun getCalculatedStdDevsFromDistanceMultipleTags(area: Double): Vector<N3> {
        val xyStdDev: Double = 0.3 / Math.pow(area, 2.0)
        val thetaStdDev = 99999 / area

        return createStdDevs(xyStdDev, xyStdDev, thetaStdDev)
    }

    fun getCalculatedStdDevs(limelight: Limelight): Vector<N3> {
        val error = getVisionPoseError(limelight)

        if (limelight.numberOfTagsInView >= 2) {
            return getCalculatedStdDevsFromDistanceMultipleTags(limelight.tagArea)
        } else if (limelight.numberOfTagsInView === 1) {
            //one tag with large area but larger pose error

            if (limelight.tagArea > 0.6 && error < 0.5) {
                return createStdDevs(3.0, 3.0, 99999.0)
            } else if (limelight.tagArea > 0.1 && error < 0.3) {
                return createStdDevs(4.0, 4.0, 99999.0)
            }
        }

        // Return default values if nothing is true
        return createStdDevs(
            Constants.PoseConfig.K_VISION_STD_DEV_X,
            Constants.PoseConfig.K_VISION_STD_DEV_Y,
            Constants.PoseConfig.K_VISION_STD_DEV_THETA
        )
    }

    val distanceToSpeaker: Double
        get() {
            val speaker: Pose2d =
                if (RobotContainer.alliance == DriverStation.Alliance.Red) Constants.RedTeamPoses.RED_SPEAKER_POSE else Constants.BlueTeamPoses.BLUE_SPEAKER_POSE
            return getDistance(speaker)
        }

    fun periodic() {
        SmartDashboard.putNumber(
            "Vision Pose Error Limelight Front",
            getVisionPoseError(SwerveSubsystem.limelightShooter)
        )
        SmartDashboard.putNumber(
            "Vision Pose Error Limelight Back",
            getVisionPoseError(SwerveSubsystem.limelightArm)
        )

        SmartDashboard.putNumber(
            if ((RobotContainer.alliance == DriverStation.Alliance.Red)) "Red speaker distance" else "Blue speaker distance",
            (if ((RobotContainer.alliance == DriverStation.Alliance.Red)) getDistance(Constants.RedTeamPoses.RED_SPEAKER_POSE) else getDistance(
                Constants.BlueTeamPoses.BLUE_SPEAKER_POSE
            ))
        )

        // ???
//        SmartDashboard.putNumber("Calculated Angle from Odometry", getPivotAngle(RobotContainer.alliance))
    }
}