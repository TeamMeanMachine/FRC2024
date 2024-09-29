package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonCamera
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.robotMode
import org.team2471.frc.lib.vision.Camera
import org.team2471.frc.lib.vision.GlobalPose
import org.team2471.frc2024.Drive.advantageWheelPoseEntry
import org.team2471.frc2024.Drive.deltaPos
import org.team2471.frc2024.Drive.driveStDevM
import org.team2471.frc2024.Drive.heading
import org.team2471.frc2024.Drive.testWheelPosition
import kotlin.math.pow

object AprilTag: Subsystem("AprilTag") {
    val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")
    val aprilTable = NetworkTableInstance.getDefault().getTable("AprilTag")
    val aprilTagsEnabledEntry = aprilTable.getEntry("AprilTags Enabled")

    val positionEntry = aprilTable.getEntry("Position")

    val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)

    val speakerTagHeight = 57.13.inches

    val excludedIDs = intArrayOf()

    var position: Vector2L = Vector2L(0.0.inches, 0.0.inches)

    var prevPosition: Vector2L = position

    var cameraPoses: MutableList<GlobalPose> = mutableListOf()

    val testKinematics: SwerveDriveKinematics = SwerveDriveKinematics(*Drive.modules.map { it.modulePosition.asMeters.toTranslation2d() }.toTypedArray())

    var modulePositions: Array<SwerveModulePosition> = Drive.modules.map { SwerveModulePosition(it.currDistance.feet.asMeters, heading.asRotation2d) }.toTypedArray()

    val poseEstimatorWPI: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(testKinematics, heading.asRotation2d, modulePositions, Pose2d())


    var aprilTagsEnabled: Boolean
        get() = aprilTagsEnabledEntry.getBoolean(true)
        set(value) {
            aprilTagsEnabledEntry.setBoolean(value)
        }

    var robotToCamSL: Transform3d = Transform3d(
        Translation3d(-6.45.inches.asMeters, 9.54.inches.asMeters, 8.75.inches.asMeters),
        Rotation3d(0.0, -65.0.degrees.asRadians, 150.0.degrees.asRadians)

//        Rotation3d(0.0, -70.degrees.asRadians, 145.0.degrees.asRadians)
//        Rotation3d(0.0, -60.degrees.asRadians, 170.0.degrees.asRadians)
    )

    var robotToCamSR = Transform3d(
        Translation3d(-6.45.inches.asMeters, -9.54.inches.asMeters, 8.75.inches.asMeters),
        Rotation3d(0.0, -65.0.degrees.asRadians, -150.0.degrees.asRadians)

//        Rotation3d(0.0, -70.degrees.asRadians, -145.0.degrees.asRadians)
//        Rotation3d(0.0.degrees.asRadians, -60.degrees.asRadians, -170.0.degrees.asRadians)
    )
    var robotToCamIB = Transform3d(
        Translation3d(12.05.inches.asMeters, 0.0.inches.asMeters, 8.0.inches.asMeters),
        Rotation3d(0.0.degrees.asRadians, -58.0.degrees.asRadians, 0.0.degrees.asRadians)
    )

    // TODO(Get the values from the limelight gui)
    var robotToCamLLShooter = Transform3d()

    val cameras: Map<String, Camera> = mapOf(
        Pair("CamSL", Camera(pvTable, aprilTable, "CamSL", aprilTagFieldLayout, robotToCamSL, robotMode, true)),
        Pair("CamSR", Camera(pvTable, aprilTable, "CamSR", aprilTagFieldLayout, robotToCamSR, robotMode, true)),
//        Pair("CamIB", Camera(pvTable, aprilTable, "CamIB", aprilTagFieldLayout, robotToCamIB, robotMode, true)),
        Pair("limelight-shooter", Camera(NetworkTableInstance.getDefault().getTable("limelight-shooter"), aprilTable, "limelight-shooter", aprilTagFieldLayout, robotToCamLLShooter, robotMode, false))
    )


    init {
        println("AprilTag init $robotMode")

        resetCameras()

        aprilTagsEnabledEntry.setBoolean(true)

        GlobalScope.launch {
            println("In periodic apriltag")
            periodic(0.02) {


                cameraPoses.clear()


                try {
                    cameras.values.forEach { cameraPoses.add(it.getEstimatedGlobalPose(Drive.position.feet, heading, Drive::lookupPose)) }
                } catch (ex: Exception) {
                    println("Error in AprilTag: $ex")
                }


                updatePos(driveStDevM, *cameraPoses.toTypedArray())

//                updatePosWPI(*cameraPoses.toTypedArray())

                positionEntry.setAdvantagePose(position, heading)
//                println("Drive Position: ${Drive.position}")\

                // In a try bc sometimes it likes to log before the table is ready :(
                try {
                    Logger.recordOutput(
                        "AprilTag/Position",
                        Pose2d(position.asMeters.toTranslation2d(), Rotation2d(heading.asRadians))
                    )

                    Logger.recordOutput("AprilTag/PositionWPI", poseEstimatorWPI.estimatedPosition)
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

    fun updatePos(driveStDevMeters: Double, vararg aprilPoses: GlobalPose) {
        val pos = position
//                                            measurement, stdev
        val measurementsAndStDevs: MutableList<Pair<Vector2L, Double>> = mutableListOf()



        testWheelPosition = pos + deltaPos
        deltaPos = Vector2L.Zeros

        advantageWheelPoseEntry.setAdvantagePose(testWheelPosition, heading)

        //add drive pose
        measurementsAndStDevs.add(Pair(testWheelPosition, driveStDevMeters))


        //add camera poses
        if (aprilTagsEnabled) {
            for (pose in aprilPoses) {
                try {
                measurementsAndStDevs.add(Pair(pose.latencyAdjustedPose(Drive.position.feet, Drive::lookupPose), pose.stDev))
                } catch (e: Exception) {
                    println("Stupid thing with latency adjust again")
                }
            }
        }

        // more into here https://docs.google.com/document/d/1FoDFRYyeyxJ-kkqUKKqm2p3dM88hWMFwhrL7IWsKgeM/edit#heading=h.src21s3e4v7y
        var totalPos = Vector2L.Zeros
        var totalStDev = 0.0

        for (posAndStDev in measurementsAndStDevs) {
            val editedStDev = posAndStDev.second.pow(-2)
            totalPos += posAndStDev.first.asMeters.times(editedStDev).meters
            totalStDev += editedStDev
        }
//    if (pos != combinedPosition) {
//        println("WAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
//    }

        //                       breaks apriltags for some reason
        if (totalStDev != 0.0 /*&& totalStDev < 1000000000.0*/) {
            position = totalPos.asMeters.div(totalStDev).meters
//        combinedPosition.coerceIn(Vector2L(0.0.inches, 0.0.inches) + Vector2L(16.0.inches, 16.0.inches), Vector2L(1654.0.cm, 821.0.cm) - Vector2L(16.0.inches, 16.0.inches))
        }
        prevPosition = pos
    }

    fun updatePosWPI(vararg aprilPoses: GlobalPose) {
//        modulePositions = Drive.modules.map { SwerveModulePosition(it.currDistance.feet.asMeters, heading.asRotation2d) }.toTypedArray()
//
//        poseEstimatorWPI.update(heading.asRotation2d, modulePositions)
//
//
//        for (pose in aprilPoses) {
//            poseEstimatorWPI.addVisionMeasurement(pose.pose2d, pose.timestampSeconds)
////             Is this how you do it???
//            poseEstimatorWPI.setVisionMeasurementStdDevs(Matrix(Nat.N3(), Nat.N1(), doubleArrayOf(pose.stDev, pose.stDev, pose.stDev)))
//        }
    }
}