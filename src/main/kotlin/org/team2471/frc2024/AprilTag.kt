package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.util.isSim
import org.team2471.frc.lib.util.robotMode
import org.team2471.frc.lib.vision.Camera
import org.team2471.frc.lib.vision.CameraType
import org.team2471.frc.lib.vision.removeTags
import org.team2471.frc2024.Drive.heading
import org.team2471.frc2024.Drive.headingRate
import org.team2471.frc2024.Drive.poseEstimator

object AprilTag : Subsystem("AprilTag") {
    val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")
    val aprilTable = NetworkTableInstance.getDefault().getTable("AprilTag")
    val aprilTagsEnabledEntry = aprilTable.getEntry("AprilTags Enabled")

    val positionPublisher = aprilTable.getStructTopic("Position", Pose2d.struct).publish()

    val excludedIDs = intArrayOf()

    val aprilTagFieldLayout: AprilTagFieldLayout =
        AprilTagFieldLayout("C:\\Users\\teamm\\IdeaProjects\\FRC2024\\src\\main\\kotlin\\org\\team2471\\frc2024\\2025-reefscape.json")

    val position: Vector2L
        get() = poseEstimator.latestPos

    var aprilTagsEnabled: Boolean
        get() = aprilTagsEnabledEntry.getBoolean(true)
        set(value) {
            aprilTagsEnabledEntry.setBoolean(value)
        }

    var robotToCamSL: Transform3d = Transform3d(
        Translation3d(-6.45.inches.asMeters, 9.54.inches.asMeters, 8.75.inches.asMeters),
        Rotation3d(0.0, -25.0.degrees.asRadians, 150.0.degrees.asRadians)

//        Rotation3d(0.0, -70.degrees.asRadians, 145.0.degrees.asRadians)
//        Rotation3d(0.0, -60.degrees.asRadians, 170.0.degrees.asRadians)
    )

    var robotToCamSR = Transform3d(
        Translation3d(-6.45.inches.asMeters, -9.54.inches.asMeters, 8.75.inches.asMeters),
        Rotation3d(0.0, -25.0.degrees.asRadians, -150.0.degrees.asRadians)

//        Rotation3d(0.0, -70.degrees.asRadians, -145.0.degrees.asRadians)
//        Rotation3d(0.0.degrees.asRadians, -60.degrees.asRadians, -170.0.degrees.asRadians)
    )
    var robotToCamIB = Transform3d(
        Translation3d(12.05.inches.asMeters, 0.0.inches.asMeters, 8.0.inches.asMeters),
        Rotation3d(0.0.degrees.asRadians, -32.0.degrees.asRadians, 0.0.degrees.asRadians)
    )

    var robotToCamLLShooter = Transform3d(
        Translation3d(-6.0.inches.asMeters, 2.5.inches.asMeters, 5.0.inches.asMeters),
        Rotation3d(0.0.degrees.asRadians, -30.degrees.asRadians, 0.0.degrees.asRadians)
    )

    val cameras: Map<String, Camera> = mapOf(
        Pair(
            "CamSR",
            Camera(pvTable, aprilTable, "CamSR", aprilTagFieldLayout, robotToCamSR, robotMode, CameraType.PHOTONVISION)
        ),
        Pair(
            "CamIB",
            Camera(pvTable, aprilTable, "CamIB", aprilTagFieldLayout, robotToCamIB, robotMode, CameraType.PHOTONVISION)
        ),
        Pair(
            "CamSL",
            Camera(pvTable, aprilTable, "CamSL", aprilTagFieldLayout, robotToCamSL, robotMode, CameraType.PHOTONVISION)
        ),
        Pair(
            "limelight-shooter",
            Camera(
                NetworkTableInstance.getDefault().getTable("limelight-shooter"),
                aprilTable,
                "limelight-shooter",
                aprilTagFieldLayout,
                robotToCamLLShooter,
                robotMode,
                CameraType.LIMELIGHT
            )
        )
    )


    init {
        println("AprilTag init $robotMode")

        resetCameras()

        aprilTagsEnabledEntry.setBoolean(true)

        GlobalScope.launch {
            println("In periodic apriltag")
            periodic(0.02) {

                cameras.values.forEach {
                    it.update(if (isSim) Drive.position.feet else position, heading, headingRate.changePerSecond)
                }

                try {
                    cameras.values.forEach {
                        poseEstimator.addVisionUpdates(it.latestPoses)
                    }
                } catch (ex: Exception) {
                    println("Error in AprilTag: ${ex.message}")
                }

                positionPublisher.setAdvantagePose(position, heading)
            }
        }
    }

    fun resetCameras() {
        PhotonCamera.setVersionCheckEnabled(false)
        println("in apriltag for reset")
        for (camera in cameras.values) {
            camera.reset()
        }
    }
}