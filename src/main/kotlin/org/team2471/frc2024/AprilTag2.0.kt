package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.MultiTargetPNPResult
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.motion.following.poseDiff
import org.team2471.frc.lib.units.*
import org.team2471.frc2024.AprilTag2.aprilTable
import org.team2471.frc2024.AprilTag2.aprilTagFieldLayout
import org.team2471.frc2024.AprilTag2.pvTable

object AprilTag2 {
    val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")
    val aprilTable = NetworkTableInstance.getDefault().getTable("AprilTag2.0")

    val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)

    var robotToCamSL: Transform3d = Transform3d(
        Translation3d(-6.45.inches.asMeters, 9.54.inches.asMeters, 8.75.inches.asMeters),
        Rotation3d(0.0, -60.degrees.asRadians, 170.0.degrees.asRadians)
    )

    var robotToCamSR = Transform3d(
        Translation3d(-6.45.inches.asMeters, -9.54.inches.asMeters, 8.75.inches.asMeters),
        Rotation3d(0.0.degrees.asRadians, -60.degrees.asRadians, -170.0.degrees.asRadians)
    )
    var robotToCamIB = Transform3d(
        Translation3d(12.05.inches.asMeters, 0.0.inches.asMeters, 8.0.inches.asMeters),
        Rotation3d(0.0.degrees.asRadians, -58.0.degrees.asRadians, 0.0.degrees.asRadians)
    )

    val cameras: Map<String, Camera> = mapOf(
        Pair("CamSL", Camera("CamSL", robotToCamSL)),
        Pair("CamSR", Camera("CamSR", robotToCamSL)),
        Pair("CamIB", Camera("CamIB", robotToCamSL))
    )

    init {
        resetCameras()
        GlobalScope.launch {
            periodic {
//                try {
                    for (camera in cameras.values) {
                        camera.getEstimatedGlobalPose(Drive.position.feet, Drive.heading )
                    }
//                } catch (ex: Exception) {
//                    println("WAAAAAAAAAAAAAA Error in apriltag: $ex")
//                }
            }
        }
    }

    fun resetCameras() {
        for (camera in cameras.values) {
            camera.reset()
        }
    }


}

class Camera(val name: String, val robotToCamera: Transform3d, val singleTagStrategy: PoseStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE, val multiTagStrategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {

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

        var newPose = if (numTargets > 1) multiTagEstimator.update() else singleTagEstimator.update()
        if (newPose.isPresent) {

            var estimatedPose = Vector2L(newPose.get().estimatedPose.x.meters, newPose.get().estimatedPose.y.meters)

            var avgDist = 0.0.inches

            for (target in validTargets) {
                val tagPose = aprilTagFieldLayout.getTagPose(target.fiducialId).get()
                avgDist += Vector2L(tagPose.x.meters, tagPose.y.meters).distance(estimatedPose)
            }

            avgDist /= validTargets.size.toDouble()

            //Todo: Get stdev from avgDist & validTargets.size

            try {
                estimatedPose = timeAdjust(estimatedPose, newPose.get().timestampSeconds)
            } catch (ex: Exception) {

            }

            advantagePoseEntry.setAdvantagePose(estimatedPose, Drive.heading)

            return GlobalPose(estimatedPose, 0.0)
        } else {
            return null
        }
    }
}

data class GlobalPose (
    val pose: Vector2L,
    val stDev: Double
)

