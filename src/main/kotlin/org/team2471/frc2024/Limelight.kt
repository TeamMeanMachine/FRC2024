package org.team2471.frc.lib.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.jetbrains.kotlin.gradle.utils.`is`
import org.photonvision.PhotonCamera
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2L
import org.team2471.frc.lib.math.setAdvantagePose
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.vision.Limelight.limelightDistCurve
import org.team2471.frc2024.AprilTag
import org.team2471.frc2024.Drive


object Limelight: Subsystem("Limelight") {


    val limelight: LimelightCamera = LimelightCamera(AprilTag.aprilTable, "limelight-shooter")

    val isConnected: Boolean
        get() = limelight.isConnected

    val limelightDistCurve = MotionCurve()


    init {
        limelight.reset()
        rebuildCurves()

        GlobalScope.launch {
            periodic {
                try {
                    if (limelight.isConnected) limelight.getEstimatedGlobalPose()
                } catch (ex: Exception) {
                    println("Error in Limelight: $ex")
                }

            }
        }
    }

    fun getCurrentGlobalPose(): GlobalPose? {
        val lastGlobalPose = limelight.lastGlobalPose
        if (lastGlobalPose != null) {
            limelight.lastGlobalPose = null
        }
        return lastGlobalPose
    }

    fun rebuildCurves() {
        limelightDistCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        // dist in meters
        limelightDistCurve.storeValue(1.5, 0.00075)
        limelightDistCurve.storeValue(1.9, 0.00125)
        limelightDistCurve.storeValue(2.5, 0.003)
        limelightDistCurve.storeValue(3.0, 0.0065)
        limelightDistCurve.storeValue(3.5, 0.0023)
        limelightDistCurve.storeValue(4.0, 0.014)
        limelightDistCurve.storeValue(4.5, 0.025) //photonvision says not to trust after 15 feet  old: 0.0165
        limelightDistCurve.storeValue(5.0, 0.03) // 0.02
        limelightDistCurve.storeValue(6.0, 0.04) // 0.03
    }

    override fun preEnable() {
        limelight.reset()
    }

    override suspend fun default() {
        periodic {
            limelight.isConnectedEntry.setBoolean(limelight.isConnected)
        }
    }
}

class LimelightCamera(
    networkTable: NetworkTable,
    name: String,
): Camera(networkTable, name) {


//    TODO(Unsure how limelight handles disconnects)
    override val isConnected: Boolean = true
//        get() = limelightTable exists

    override fun reset() {
        try {
            println("Implement reset")
        } catch (ex: Exception) {
            println("Error resetting cam $name: $ex")
        }
    }


    override fun getEstimatedGlobalPose(): GlobalPose? {
        val poseArray = LimelightHelpers.getBotPoseEstimate_wpiBlue(name)   // LimelightHelpers.getBotPose3d_wpiBlue(name)
        var estimatedPose = Vector2L(poseArray.pose.translation.x.meters, poseArray.pose.translation.y.meters)
        var tagCount = poseArray.tagCount

        if (estimatedPose == Vector2L(0.0.meters, 0.0.meters) || tagCount < 2) return null // estimatedPose returns origin if no tags seen

        var avgDist = 0.0.inches

        for (target in poseArray.rawFiducials!!) {
            avgDist += target?.distToCamera?.meters ?: 0.0.inches
        }
        avgDist /= poseArray.tagCount.toDouble()

        var stDev = limelightDistCurve.getValue(avgDist.asMeters)

        lastGlobalPose = GlobalPose(estimatedPose, Drive.heading, stDev, Timer.getFPGATimestamp())

        stDevEntry.setDouble(stDev)

        advantagePoseEntry.setAdvantagePose(estimatedPose, Drive.heading)
        // Todo filter more

        return lastGlobalPose
    }
}