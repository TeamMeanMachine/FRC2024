package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.MultiTargetPNPResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.*
import org.team2471.frc2024.AprilTag.aprilTable
import org.team2471.frc2024.AprilTag.aprilTagFieldLayout
import org.team2471.frc2024.AprilTag.distCurve
import org.team2471.frc2024.AprilTag.pvTable
import org.team2471.frc2024.Drive.isRedAlliance

object AprilTag {
    val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")
    val aprilTable = NetworkTableInstance.getDefault().getTable("AprilTag2.0")
    val aprilTagsEnabledEntry = aprilTable.getEntry("AprilTags Enabled")

    val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)

    val speakerTagHeight = 57.13.inches

    var last2DSpeakerDist = LinearFilter.movingAverage(5)
    var last2DSpeakerAngle = LinearFilter.movingAverage(5)

    var aprilTagsEnabled: Boolean
        get() = aprilTagsEnabledEntry.getBoolean(true)
        set(value) {
            aprilTagsEnabledEntry.setBoolean(value)
        }

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
        Pair("CamSR", Camera("CamSR", robotToCamSR)),
        Pair("CamIB", Camera("CamIB", robotToCamIB))
    )

    val distCurve = MotionCurve()

    init {
        resetCameras()
        rebuildCurves()

        last2DSpeakerDist.reset()
        last2DSpeakerAngle.reset()
        last2DSpeakerAngle.calculate(0.0)
        last2DSpeakerDist.calculate(0.0)

        GlobalScope.launch {
            periodic {

                val temp2DOffset = get2DSpeakerOffset()
                if (temp2DOffset != null) {
//                    println("Setting angle to ${temp2DOffset.second}")
                    last2DSpeakerDist.calculate(temp2DOffset.first.asFeet)
                    last2DSpeakerAngle.calculate(-temp2DOffset.second.asDegrees)
                }

                try {
                    for (camera in cameras.values) {
                        if (camera.photonCam.isConnected) {
                            camera.getEstimatedGlobalPose(Drive.position.feet, Drive.heading )
                        }
                    }
                } catch (ex: Exception) {
                    println("Error in apriltag: $ex")
                }
            }
        }
    }

    fun resetCameras() {
        for (camera in cameras.values) {
            camera.reset()
        }
    }

    fun getCurrentGlobalPoses(): Array<GlobalPose> {
        val out: MutableList<GlobalPose> = mutableListOf()

        for (camera in cameras.values) {
            val lastGlobalPose = camera.lastGlobalPose
            if (lastGlobalPose != null) {
                if (Timer.getFPGATimestamp() - lastGlobalPose.timestamp < 0.03) {
                    out.add(lastGlobalPose)
                }
            }
        }

        return out.toTypedArray()
    }

    fun get2DSpeakerOffset(): Pair<Length, Angle>? {
        try {
            if (cameras["CamSL"] == null || cameras["CamSR"] == null) {
                return null
            } else if (!(cameras["CamSL"]?.photonCam?.isConnected ?: false) || !(cameras["CamSR"]?.photonCam?.isConnected ?: false)) {
                return null
            } else {
                var slDist = 0.0.feet
                var srDist = 0.0.feet
                var slRot = 0.0.degrees
                var srRot = 0.0.degrees
                var numCams = 0.0

                val camSL = cameras["CamSL"]?.photonCam
                val camSR = cameras["CamSR"]?.photonCam

                if (camSL != null) {
                    if (camSL.isConnected) {
                        val validSLTargets = camSL.latestResult.targets
                        for (target in validSLTargets) {
                            if (target.fiducialId == if (isRedAlliance) 4 else 7) {
                                if (target.area > 5.0){
                                    //                    println("pitch SL: ${-robotToCamSL.rotation.y.radians}")
                                    slDist =
                                        (speakerTagHeight - robotToCamSL.z.meters) / Angle.tan(-robotToCamSL.rotation.y.radians + target.pitch.degrees)
                                    //                    println("slDist: ${slDist}")
                                    slRot = (10.0.degrees) + target.yaw.degrees
                                    numCams += 1.0
                                }
                            }
                        }
                    }

                }
                if (camSR != null) {
                    if (camSR.isConnected) {
                        val validSRTargets = camSR.latestResult.targets
                        for (target in validSRTargets) {
                            if (target.fiducialId == if (isRedAlliance) 4 else 7) {
                                if (target.area > 5.0) {

                                    //                    println("pitch SR: ${-robotToCamSR.rotation.y.radians}")
                                    srDist =
                                        (speakerTagHeight - robotToCamSR.z.meters) / Angle.tan(-robotToCamSR.rotation.y.radians + target.pitch.degrees)
                                    srRot = ((-10.0).degrees) + target.yaw.degrees
                                    numCams += 1.0
                                    //                    println("sr =dist ${srDist}")
                                }
                            }
                        }
                    }
                }

                return Pair((slDist.asFeet.meters + srDist.asFeet.meters) / numCams, slRot + srRot / numCams)
            }
        } catch (ex: Exception) {
            println("2D FAILED!!!!!!!!! $ex")
            return null
        }
    }

    fun rebuildCurves() {
        distCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        // dist in meters
        distCurve.storeValue(1.5, 0.00075)
        distCurve.storeValue(1.9, 0.00125)
        distCurve.storeValue(2.5, 0.003)
        distCurve.storeValue(3.0, 0.0065)
        distCurve.storeValue(3.5, 0.0023)
        distCurve.storeValue(4.0, 0.014)
        distCurve.storeValue(4.5, 0.0165)
        distCurve.storeValue(5.0, 0.02)
        distCurve.storeValue(6.0, 0.03)
    }
}

class Camera(val name: String, val robotToCamera: Transform3d, val singleTagStrategy: PoseStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE, val multiTagStrategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {

    val advantagePoseEntry = aprilTable.getEntry("April Advantage Pos $name")
    val targetPoseEntry = aprilTable.getEntry("April Target Pos $name")
    val stDevEntry = aprilTable.getEntry("stDev $name")

    var lastGlobalPose: GlobalPose? = null

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
                    singleTagEstimator = PhotonPoseEstimator(
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
            var targetPoses : Array<Vector2L> = arrayOf()
            for (target in validTargets) {
                val tagPose = aprilTagFieldLayout.getTagPose(target.fiducialId).get()
                avgDist += Vector2L(tagPose.x.meters, tagPose.y.meters).distance(estimatedPose)
                val targetRelativePose = (target.bestCameraToTarget + robotToCamera).translation.toTranslation2d().rotateBy(Rotation2d(Drive.heading.asRadians))


                lastGlobalPose?.pose?.plus(Vector2L(targetRelativePose.x.meters, targetRelativePose.y.meters))
                    ?.let { targetPoses += it }
            }
            targetPoseEntry.setAdvantagePoses(targetPoses)
            avgDist /= validTargets.size.toDouble()

            var stDev = distCurve.getValue(avgDist.asMeters)

            if (numTargets < 2) stDev *= 10.0
            if (name == "CamSL") stDev *= 5.0

            try {
                estimatedPose = timeAdjust(estimatedPose, newPose.get().timestampSeconds)
            } catch (_: Exception) {

            }

            estimatedPose.coerceIn(Vector2L(0.0.inches, 0.0.inches), Vector2L(1654.0.cm, 821.0.cm))

            advantagePoseEntry.setAdvantagePose(estimatedPose, Drive.heading)

//            println("cam: ${photonCam.name} avgDist: ${avgDist.asMeters}   stDev: $stDev")

            lastGlobalPose = GlobalPose(estimatedPose, stDev, Timer.getFPGATimestamp())

            stDevEntry.setDouble(stDev)

            return lastGlobalPose
        } else {
            return null
        }
    }
}

data class GlobalPose (
    val pose: Vector2L,
    val stDev: Double,
    val timestamp: Double
)

