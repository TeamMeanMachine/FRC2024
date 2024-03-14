package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.MultiTargetPNPResult
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.Vector2L
import org.team2471.frc.lib.math.asMeters
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.motion.following.poseDiff
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.meters
import org.team2471.frc2024.AprilTag2.aprilTable
import org.team2471.frc2024.AprilTag2.aprilTagFieldLayout
import org.team2471.frc2024.AprilTag2.pvTable

object AprilTag2 {
    val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")
    val aprilTable = NetworkTableInstance.getDefault().getTable("AprilTag2.0")

    val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)



}

class camera(val name: String, val robotToCamera: Transform3d, val singleTagStrategy: PoseStrategy, val multiTagStrategy: PoseStrategy) {

    val advantagePoseEntry = aprilTable.getEntry("April Advantage Pos $name")

    var photonCam: PhotonCamera = PhotonCamera(name)

    var singleTagEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
        aprilTagFieldLayout,
        multiTagStrategy,
        photonCam,
        robotToCamera
    )
    var multiTagEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
        aprilTagFieldLayout,
        singleTagStrategy,
        photonCam,
        robotToCamera
    )


    fun reset() {
        if (!photonCam.isConnected) {
            try {
                if (pvTable.containsSubTable(name)) {
                    photonCam = PhotonCamera(name)
                    multiTagEstimator = PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        multiTagStrategy,
                        photonCam,
                        robotToCamera
                    )
                    AprilTag.singleTagSLPoseEstimator = PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        singleTagStrategy,
                        photonCam,
                        robotToCamera
                    )
                } else {
                    println("Cam $name not found")
                }
            } catch (ex: Exception) {
                println("Error resetting cam $name: $ex")
            }
        } else  {
            println("$name already found, skipping reset")
        }
    }

    fun getEstimatedGlobalPose(currentPos: Vector2L, currentHeading: Angle): GlobalPose? {
        if (!photonCam.isConnected) {
            return null
        }

        val multiTagCameraResult: MultiTargetPNPResult = photonCam.latestResult.multiTagResult

        val validTargets = photonCam.latestResult.targets

        validTargets ?: return null

        val numTargets = validTargets.count()

        for (target in validTargets) {
            if (target.fiducialId > 16) {
                println("AprilTag: Invalid Tag")
                return null
            }
        }

        if (numTargets > 1) {
            multiTagEstimator.setReferencePose(
                Pose2d(
                    Translation2d(currentPos.asMeters.x, currentPos.asMeters.y),
                    Rotation2d(currentHeading.asRadians)
                )
            )
        } else {
            singleTagEstimator.setReferencePose(
                Pose2d(
                    Translation2d(currentPos.asMeters.x, currentPos.asMeters.y),
                    Rotation2d(currentHeading.asRadians)
                )
            )
        }

        val newPose = if (numTargets > 1) multiTagEstimator.update() else singleTagEstimator.update()

        val result = newPose.get()

        return GlobalPose(Vector2L(result.estimatedPose.toPose2d().x.meters, result.estimatedPose.toPose2d().y.meters), 1.0)
    }
}

data class GlobalPose (
    val pose: Vector2L,
    val stDev: Double,
)

