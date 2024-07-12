// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.lib.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.lib.util.LimelightHelpers.getBotPose2d_wpiBlue
import frc.lib.util.LimelightHelpers.getCurrentPipelineIndex
import frc.lib.util.LimelightHelpers.getLatency_Capture
import frc.lib.util.LimelightHelpers.getLatency_Pipeline
import frc.lib.util.LimelightHelpers.getLatestResults
import frc.lib.util.LimelightHelpers.getTA
import frc.lib.util.LimelightHelpers.getTV
import frc.lib.util.LimelightHelpers.setLEDMode_ForceBlink
import frc.lib.util.LimelightHelpers.setLEDMode_ForceOff
import frc.lib.util.LimelightHelpers.setPipelineIndex

// import frc.lib.util.LimelightHelpers;
class Limelight(
    /** Creates a new Limelight.  */
    val limelightName: String
) {
    private var isFlashing = false

    init {
        limelightsInUse.add(this) //adds limelight object's memory address
    }

    // //For debugging and startup purposes
    fun flashLimelight() {
        if (isFlashing) return

        Thread {
            isFlashing = true
            setLEDMode_ForceBlink(this.limelightName)
            println(this.limelightName + " IS WORKING")

            try {
                Thread.sleep(1000)
            } catch (e: Exception) {
                e.printStackTrace()
            }
            setLEDMode_ForceOff(this.limelightName)
            isFlashing = false
        }.start()
    }

    var pipeline: Int
        get() = getCurrentPipelineIndex(this.limelightName).toInt()
        set(pipeline) {
            setPipelineIndex(this.limelightName, pipeline)
        }

    val limelightLatency: Double
        get() = Timer.getFPGATimestamp() - (getLatency_Pipeline(this.limelightName) / 1000) - (getLatency_Capture(
            this.limelightName
        ) / 1000)

    val tagArea: Double
        get() = getTA(this.limelightName)

    val numberOfTagsInView: Int
        get() = getLatestResults(this.limelightName).targetingResults.targets_Fiducials.size

    val visionPredictedRobotPose: Pose2d?
        get() {
            //   LimelightHelpers.getLatestResults(this.limelightName);
            if (getTV(this.limelightName)) {
                return getBotPose2d_wpiBlue(this.limelightName)
            }

            return null
        }

    fun periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber(
            this.limelightName + " Number of Tags in View",
            numberOfTagsInView.toDouble()
        )
        SmartDashboard.putNumber(this.limelightName + " Latency", this.limelightLatency)
        SmartDashboard.putNumber(this.limelightName + " current pipeline", pipeline.toDouble())
        SmartDashboard.putString("Limelights in Use", limelightsInUse.toString())
    }

    companion object {
        val limelightsInUse: ArrayList<Limelight> = ArrayList()
    }
}