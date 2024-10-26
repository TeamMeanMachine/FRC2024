package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.robotMode
import org.team2471.frc.lib.vision.Camera
import org.team2471.frc.lib.vision.GlobalPose
import org.team2471.frc.lib.vision.getPos
import org.team2471.frc2024.Drive.heading
import org.team2471.frc2024.Drive.poseEstimator

object AprilTag: Subsystem("AprilTag") {
    val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")
    val aprilTable = NetworkTableInstance.getDefault().getTable("AprilTag")
    val aprilTagsEnabledEntry = aprilTable.getEntry("AprilTags Enabled")

    val positionPublisher = aprilTable.getStructTopic("Position", Pose2d.struct).publish()

    val wpiStates: StructArrayPublisher<SwerveModuleState> = aprilTable.getStructArrayTopic("Test States", SwerveModuleState.struct).publish()

    val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)

    val speakerTagHeight = 57.13.inches

    val excludedIDs = intArrayOf()


    val position: Vector2L
        get() = poseEstimator.getPos()

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

    // TODO(Get the values from the limelight gui)
    var robotToCamLLShooter = Transform3d()

    val cameras: Map<String, Camera> = mapOf(
        Pair("CamSL", Camera(pvTable, aprilTable, "CamSL", aprilTagFieldLayout, robotToCamSL, robotMode, true)),
        Pair("CamSR", Camera(pvTable, aprilTable, "CamSR", aprilTagFieldLayout, robotToCamSR, robotMode, true)),
        Pair("CamIB", Camera(pvTable, aprilTable, "CamIB", aprilTagFieldLayout, robotToCamIB, robotMode, true)),
//        Pair("limelight-shooter", Camera(NetworkTableInstance.getDefault().getTable("limelight-shooter"), aprilTable, "limelight-shooter", aprilTagFieldLayout, robotToCamLLShooter, robotMode, false))
    )


    init {
        println("AprilTag init $robotMode")

        resetCameras()

        aprilTagsEnabledEntry.setBoolean(true)

        GlobalScope.launch {
            println("In periodic apriltag")
            periodic(0.02) {

                try {
                    cameras.values.forEach {
                        val pose = it.getEstimatedGlobalPose(Drive.position.feet, heading, Drive::lookupPose)
                        poseEstimator.addVisionMeasurement(pose.pose2d, pose.timestampSeconds, VecBuilder.fill(pose.stDev, pose.stDev, 50.0.degrees.asRadians))
                    }
                } catch (ex: Exception) {
                    println("Error in AprilTag: $ex")
                }

//                updatePosWPI(*cameraPoses.toTypedArray())

                positionPublisher.set(poseEstimator.estimatedPosition)

                wpiStates.set(Drive.modules.map { it.wpiState }.toTypedArray())
//                println("Drive Position: ${Drive.position}")\

                // In a try bc sometimes it likes to log before the table is ready :(
                try {
//                    Logger.recordOutput(
//                        "AprilTag/Position",
//                        Pose2d(position.asMeters.toTranslation2d(), Rotation2d(heading.asRadians))
//                    )

//                    Logger.recordOutput("AprilTag/PositionWPI", poseEstimatorWPI.estimatedPosition)
                } catch (_: Exception) {}
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