package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.MultiTargetPNPResult
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.vision.Camera
import org.team2471.frc.lib.vision.GlobalPose
import org.team2471.frc.lib.vision.PhotonVisionCamera
import org.team2471.frc2024.Drive.isRedAlliance
import kotlin.math.abs
import kotlin.math.pow

object AprilTag: Subsystem("AprilTag") {
    val aprilTable: NetworkTable = NetworkTableInstance.getDefault().getTable("AprilTag2.0")
    val aprilTagsEnabledEntry: NetworkTableEntry = aprilTable.getEntry("AprilTags Enabled")

    val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)

    val speakerTagHeight = 57.13.inches

    var last2DSpeakerDistFilter = LinearFilter.movingAverage(5)
    var last2DSpeakerDist = 0.0
    var last2DSpeakerAngleFilter = LinearFilter.movingAverage(5)
    var last2DSpeakerAngle = 0.0

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
        Pair("CamSL", PhotonVisionCamera(
            aprilTable,
            "CamSL",
            robotToCamSL,
            aprilTagFieldLayout
        )),
        Pair("CamSR", PhotonVisionCamera(
            aprilTable,
            "CamSR",
            robotToCamSR,
            aprilTagFieldLayout
        )),
        Pair("CamIB", PhotonVisionCamera(
            aprilTable,
            "CamIB",
            robotToCamIB,
            aprilTagFieldLayout
        ))
    )

    val backCamsConnected: Boolean
        get() = cameras["CamSL"]?.isConnected == true && cameras["CamSR"]?.isConnected == true

    val distCurve = MotionCurve()

    init {
        resetCameras()
        rebuildCurves()

        last2DSpeakerDistFilter.reset()
        last2DSpeakerAngleFilter.reset()
        last2DSpeakerAngleFilter.calculate(0.0)
        last2DSpeakerDistFilter.calculate(0.0)

        aprilTagsEnabledEntry.setBoolean(true)

        GlobalScope.launch {
            periodic {

                val temp2DOffset = get2DSpeakerOffset()
                if (temp2DOffset != null) {
//                    println("Setting angle to ${temp2DOffset.second}")
                    last2DSpeakerDist = last2DSpeakerDistFilter.calculate(temp2DOffset.first.asFeet)
                    last2DSpeakerAngle = last2DSpeakerDistFilter.calculate(-temp2DOffset.second.asDegrees)
                }

                try {
                    for (camera in cameras.values) {
                        if (camera.isConnected) {
                            camera.getEstimatedGlobalPose(Pose2d(Translation2d(Drive.position.x, Drive.position.y), Rotation2d(Drive.heading.asRadians)), distCurve)?.latencyAdjust()
                        }
                    }
                } catch (ex: Exception) {
                    println("Error in apriltag: $ex")
                }
            }
        }
    }

    fun resetCameras() {
        PhotonCamera.setVersionCheckEnabled(false)
        for (camera in cameras.values) {
            camera.reset()
        }
    }

    fun getCurrentGlobalPoses(): Array<GlobalPose> {
        val out: MutableList<GlobalPose> = mutableListOf()

        for (camera in cameras.values) {
            val lastGlobalPose = camera.lastGlobalPose
            if (lastGlobalPose != null) {
                out.add(lastGlobalPose)
                camera.lastGlobalPose = null
            }
        }

        return out.toTypedArray()
    }

    fun get2DSpeakerOffset(): Pair<Length, Angle>? {
        try {
            if (cameras["CamSL"] == null || cameras["CamSR"] == null) {
                return null
            } else if (cameras["CamSL"]?.isConnected != true || cameras["CamSR"]?.isConnected != true) {
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
                                    numCams ++
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
                                    numCams ++
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
        distCurve.storeValue(4.5, 0.025) //photonvision says not to trust after 15 feet  old: 0.0165
        distCurve.storeValue(5.0, 0.03) // 0.02
        distCurve.storeValue(6.0, 0.04) // 0.03
    }

    override fun preEnable() {
        GlobalScope.launch {
            resetCameras()
        }
    }

    override suspend fun default() {
        periodic {
            for (c in cameras) {
                val camera = c.value
                camera.isConnectedEntry.setBoolean(camera.isConnected)
            }
        }
    }
}

