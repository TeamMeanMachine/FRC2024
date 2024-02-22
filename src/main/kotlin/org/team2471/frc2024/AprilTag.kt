package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonUtils
import org.photonvision.targeting.MultiTargetPNPResult
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.units.*
import org.team2471.frc2024.AprilTag.aprilTagFieldLayout
import org.team2471.frc2024.AprilTag.camIB
import org.team2471.frc2024.AprilTag.camSL
import org.team2471.frc2024.AprilTag.camSR
import org.team2471.frc2024.AprilTag.iBPoseEstimator
import org.team2471.frc2024.AprilTag.lastIBAmbiguity
import org.team2471.frc2024.AprilTag.lastIBDetectionTime
import org.team2471.frc2024.AprilTag.lastSLAmbiguity
import org.team2471.frc2024.AprilTag.lastSLDetectionTime
import org.team2471.frc2024.AprilTag.lastSRAmbiguity
import org.team2471.frc2024.AprilTag.lastSRDetectionTime
import org.team2471.frc2024.AprilTag.pvTable
import org.team2471.frc2024.AprilTag.robotToCamIB
import org.team2471.frc2024.AprilTag.robotToCamSL
import org.team2471.frc2024.AprilTag.robotToCamSR
import org.team2471.frc2024.AprilTag.sLPoseEstimator
import org.team2471.frc2024.AprilTag.sRPoseEstimator
import kotlin.math.absoluteValue


object AprilTag {
    val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val advantagePoseSLEntry = pvTable.getEntry("April Advantage Pose ShootLeft")
    private val advantagePoseSREntry = pvTable.getEntry("April Advantage Pose ShootRight")
    private val advantagePoseIBEntry = pvTable.getEntry("April Advantage Pose IntakeBW")
    private val seesAprilTagEntry = pvTable.getEntry("Sees an April Tag")

    val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)

//     Camera Shooter Left
    var camSL: PhotonCamera? = null
//     Camera Shooter Right
    var camSR: PhotonCamera? = null
//     Camera Intake B/W
    var camIB: PhotonCamera? = null

    var sLPoseEstimator : PhotonPoseEstimator? = null
    var sRPoseEstimator : PhotonPoseEstimator? = null
    var iBPoseEstimator : PhotonPoseEstimator? = null
    const val maxAmbiguity = 0.1
    var lastSLPose = Pose2d(0.0,0.0, Rotation2d(0.0))
    var lastSRPose = Pose2d(0.0,0.0, Rotation2d(0.0))
    var lastIBPose = Pose2d(0.0,0.0, Rotation2d(0.0))

    var lastSLDetectionTime = 0.0
    var lastSRDetectionTime = 0.0
    var lastIBDetectionTime = 0.0

    var lastSLAmbiguity = 0.0
    var lastSRAmbiguity = 0.0
    var lastIBAmbiguity = 0.0


// TODO: Test Single Tags at different distances to find the min Dist.
    private var singleTagMinDist: Double = 17.35


    val lastSLDetection: AprilDetection
        get() = AprilDetection(lastSLDetectionTime, lastSLPose, lastSLAmbiguity)

    val lastSRDetection: AprilDetection
        get() = AprilDetection(lastSRDetectionTime, lastSRPose, lastSRAmbiguity)
    val lastIBDetection: AprilDetection
        get() = AprilDetection(lastIBDetectionTime, lastIBPose, lastIBAmbiguity)

    var robotToCamSL: Transform3d = Transform3d(
        Translation3d(-6.45.inches.asMeters, -9.54.inches.asMeters, 9.0.inches.asMeters),
        Rotation3d(0.0, -50.degrees.asRadians, -170.0.degrees.asRadians)
    )

    var robotToCamSR = Transform3d(
        Translation3d(-6.45.inches.asMeters, 9.54.inches.asMeters, 9.0.inches.asMeters),
        Rotation3d(7.0.degrees.asRadians, -50.degrees.asRadians, 170.0.degrees.asRadians)
    )
    var robotToCamIB = Transform3d(
        Translation3d(12.05.inches.asMeters, 0.0.inches.asMeters, 8.0.inches.asMeters),
        Rotation3d(0.0.degrees.asRadians, -58.0.degrees.asRadians, 0.0.degrees.asRadians)
    )
    init {
        resetCameras()
        GlobalScope.launch {
            periodic {
                    try {
                        //val frontCamSelected = useFrontCam()
                        var maybePoseSL: Pose2d? =
                            sLPoseEstimator?.let { camSL?.let { it1 -> getEstimatedGlobalPose(it1, it) } }
                        var numTargetSL: Int = camSL?.latestResult?.targets?.count() ?: 0

                        var maybePoseSR: Pose2d? =
                            sRPoseEstimator?.let { camSR?.let { it1 -> getEstimatedGlobalPose(it1, it) } }
                        var numTargetSR: Int = camSR?.latestResult?.targets?.count() ?: 0

                        var maybePoseIB: Pose2d? =
                            iBPoseEstimator?.let { camIB?.let { it1 -> getEstimatedGlobalPose(it1, it) } }
                        var numTargetIB: Int = camIB?.latestResult?.targets?.count() ?: 0

                        if (maybePoseSL != null) {
                                seesAprilTagEntry.setBoolean(numTargetSL > 0)
                                advantagePoseSLEntry.setDoubleArray(
                                    doubleArrayOf(
                                        maybePoseSL.x,
                                        maybePoseSL.y,
                                        maybePoseSL.rotation.degrees
                                    )
                                )
                            lastSLPose = maybePoseSL
                            PoseEstimator.addVision(lastSLDetection, numTargetSL)
                        }
                        if (maybePoseSR != null) {
                                seesAprilTagEntry.setBoolean(numTargetSR > 0)
                                advantagePoseSREntry.setDoubleArray(
                                    doubleArrayOf(
                                        maybePoseSR.x,
                                        maybePoseSR.y,
                                        maybePoseSR.rotation.degrees
                                    )
                                )
                            lastSRPose = maybePoseSR

                            PoseEstimator.addVision(lastSRDetection, numTargetSR)
                        }
                        if (maybePoseIB != null) {
                            seesAprilTagEntry.setBoolean(numTargetIB > 0)
                            advantagePoseIBEntry.setDoubleArray(
                                doubleArrayOf(
                                    maybePoseIB.x,
                                    maybePoseIB.y,
                                    maybePoseIB.rotation.degrees
                                )
                            )
                            lastIBPose = maybePoseIB


                            PoseEstimator.addVision(lastIBDetection, numTargetIB)
                        }
                    } catch (ex:Exception) {
                        println("Error in apriltag $ex")
                    }
                }
            }
        }
    }

    private fun getEstimatedGlobalPose(camera: PhotonCamera, estimator: PhotonPoseEstimator): Pose2d? {

        try {
            if (!camera.isConnected) {
                return null
            }
            val cameraResult: MultiTargetPNPResult? = camera.latestResult.multiTagResult
            val validTargets = cameraResult?.fiducialIDsUsed//.filter{ validTags.contains(it.fiducialId) && it.poseAmbiguity < maxAmbiguity }
            if (validTargets != null) {
                if (validTargets.isEmpty()) {
                    // println("AprilTag: Empty Tags")
                    return null
                }
            } else {
                return null
            }
            for (target in validTargets) {
                if (target > 16) {
                    println("AprilTag: Invalid Tag")
                    return null
                }
            }
            if ((validTargets.count() < 2 && cameraResult.estimatedPose.ambiguity > 0.05) || cameraResult.estimatedPose.ambiguity > 0.15) {
                return null
            }
            //println("at least 2 valid targets found ${poseList}")
            estimator.setReferencePose(
                Pose2d(
                    cameraResult.estimatedPose.best.translation.toTranslation2d(),
                    cameraResult.estimatedPose.best.rotation.toRotation2d()
                )
            )

            val newPose = estimator.update()

//                println("newPose: $newPose")
            if (newPose?.isPresent == true) {
                val result = newPose.get()
                //TODO: filter single tags by distance
//                if (validTargets.count() < 2 && result.s) {

//                println("AprilTag: Single target too far away ${result.estimatedPose.toPose2d().toTMMField().y.absoluteValue} vs ${(FieldManager.chargeFromCenterY + FieldManager.chargingStationDepth).asFeet}")
//                    return null
//                }
//            println("Valid target found ${validTargets.count()}")
                when (camera.name) {
                    "CamSL" -> {lastSLDetectionTime = result.timestampSeconds; lastSLAmbiguity = cameraResult.estimatedPose.ambiguity}
                    "CamSR" -> {lastSRDetectionTime = result.timestampSeconds; lastSRAmbiguity = cameraResult.estimatedPose.ambiguity}
                    "CamIB" -> {lastIBDetectionTime = result.timestampSeconds; lastIBAmbiguity = cameraResult.estimatedPose.ambiguity}
                }


                return result.estimatedPose.toPose2d()
            } else {

                return null
            }
        } catch (ex: Exception) {
            println("***********************************************************AprilTag Failed. Try Operator down*****************************************************")
            return null
        }
    }

    fun resetCameras() {
        if (camSL == null || sLPoseEstimator == null) {
            try {
                if (pvTable.containsSubTable("CamSL")) {
                    camSL = PhotonCamera("CamSL")
                    sLPoseEstimator = PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        camSL,
                        robotToCamSL
                    )
                }
            } catch (ex: Exception) {
                println("sL pose failed")
            }
        } else {
            println("CamSL already found, skipping reset")
        }
        if (camSR == null || sRPoseEstimator == null) {
            try {
                if (pvTable.containsSubTable("CamSR")) {
                    camSR = PhotonCamera("CamSR")
                    sRPoseEstimator = PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        camSR,
                        robotToCamSR
                    )
                }
            } catch (ex: Exception) {
                println("SR pose failed")
            }
        } else {
            println("CamSR already found, skipping reset")
        }
        println("Finished cams reset")

        if (camIB == null || iBPoseEstimator == null) {
            try {
                if (pvTable.containsSubTable("CamIB")) {
                    camIB = PhotonCamera("CamIB")
                    iBPoseEstimator = PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        camIB,
                        robotToCamIB
                    )
                }
            } catch (ex: Exception) {
                println("IB pose failed")
            }
        } else {
            println("CamIB already found, skipping reset")
        }
    }

data class AprilDetection (
    val timestamp: Double,
    val pose: Pose2d,
    val ambiguity: Double
)