package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.MultiTargetPNPResult
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.units.Angle.Companion.tan
import org.team2471.frc2024.AprilTag.aprilTagFieldLayout
import org.team2471.frc2024.AprilTag.singleTagSLPoseEstimator
import org.team2471.frc2024.AprilTag.singleTagSRPoseEstimator
import org.team2471.frc2024.AprilTag.camIB
import org.team2471.frc2024.AprilTag.camSL
import org.team2471.frc2024.AprilTag.camSR
import org.team2471.frc2024.AprilTag.excludedTagsList
import org.team2471.frc2024.AprilTag.iBPoseEstimator
import org.team2471.frc2024.AprilTag.lastIBAmbiguity
import org.team2471.frc2024.AprilTag.lastIBDetectionTime
import org.team2471.frc2024.AprilTag.lastIBDist
import org.team2471.frc2024.AprilTag.lastSLAmbiguity
import org.team2471.frc2024.AprilTag.lastSLDetectionTime
import org.team2471.frc2024.AprilTag.lastSLDist
import org.team2471.frc2024.AprilTag.lastSRAmbiguity
import org.team2471.frc2024.AprilTag.lastSRDetectionTime
import org.team2471.frc2024.AprilTag.lastSRDist
import org.team2471.frc2024.AprilTag.pvTable
import org.team2471.frc2024.AprilTag.robotToCamIB
import org.team2471.frc2024.AprilTag.robotToCamSL
import org.team2471.frc2024.AprilTag.robotToCamSR
import org.team2471.frc2024.AprilTag.multiTagSLPoseEstimator
import org.team2471.frc2024.AprilTag.multiTagSRPoseEstimator
import org.team2471.frc2024.AprilTag.singleTagMinDist
import org.team2471.frc2024.AprilTag.speakerTagHeight


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

    var multiTagSLPoseEstimator : PhotonPoseEstimator? = null
    var multiTagSRPoseEstimator : PhotonPoseEstimator? = null
    var singleTagSLPoseEstimator : PhotonPoseEstimator? = null
    var singleTagSRPoseEstimator : PhotonPoseEstimator? = null
    var iBPoseEstimator : PhotonPoseEstimator? = null
    const val maxAmbiguity = 0.1
    var distFromSpeaker: Length = 0.0.feet

    val speakerTagHeight = 57.13.inches

    var lastSLPose = Pose2d(0.0,0.0, Rotation2d(0.0))
    var lastSRPose = Pose2d(0.0,0.0, Rotation2d(0.0))
    var lastIBPose = Pose2d(0.0,0.0, Rotation2d(0.0))

    var lastSLDetectionTime = 0.0
    var lastSRDetectionTime = 0.0
    var lastIBDetectionTime = 0.0

    var lastSLAmbiguity = 0.0
    var lastSRAmbiguity = 0.0
    var lastIBAmbiguity = 0.0

    var lastSLDist = 0.0.feet
    var lastSRDist = 0.0.feet
    var lastIBDist = 0.0.feet

    var excludedTagsList: List<Int> = listOf() //EXCLUDE APRILTAGS HERE!!!!!!!!!


    // TODO: Test Single Tags at different distances to find the min Dist.
    var singleTagMinDist: Double = 12.0 //ft
    private var multiTagMinDist: Double = 16.0


    val lastSLDetection: AprilDetection
        get() = AprilDetection(lastSLDetectionTime, lastSLPose, lastSLDist, lastSLAmbiguity)

    val lastSRDetection: AprilDetection
        get() = AprilDetection(lastSRDetectionTime, lastSRPose, lastSRDist, lastSRAmbiguity)
    val lastIBDetection: AprilDetection
        get() = AprilDetection(lastIBDetectionTime, lastIBPose, lastIBDist, lastIBAmbiguity)


    var last2DSpeakerDist: Length = 0.0.feet
    var  last2DSpeakerAngle: Angle = 0.0.degrees

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
    init {
        resetCameras()
        GlobalScope.launch {
            periodic {
                val temp2DOffset = get2DSpeakerOffset()
                if (temp2DOffset != null) {
//                    println("Setting angle to ${temp2DOffset.second}")
                    last2DSpeakerDist = temp2DOffset.first
                    last2DSpeakerAngle = -temp2DOffset.second
                }
//                println("2d: ${get2DSpeakerOffset()}")
                try {
                    //val frontCamSelected = useFrontCam()
                    val numTargetSL: Int = camSL?.latestResult?.targets?.count() ?: 0
//                    println(numTargetSL)
                    val maybePoseSL: Pose2d? =
                        singleTagSLPoseEstimator?.let { camSL?.let { it1 -> getEstimatedGlobalPose(it1, numTargetSL, it, multiTagSLPoseEstimator) } }

                    val numTargetSR: Int = camSR?.latestResult?.targets?.count() ?: 0
                    val maybePoseSR: Pose2d? =
                        singleTagSRPoseEstimator?.let { camSR?.let { it1 -> getEstimatedGlobalPose(it1, numTargetSR, it, multiTagSRPoseEstimator) } }

                    val numTargetIB: Int = camIB?.latestResult?.targets?.count() ?: 0
                    val maybePoseIB: Pose2d? =
                        iBPoseEstimator?.let { camIB?.let { it1 -> getEstimatedGlobalPose(it1, numTargetIB, it) } }

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
                    } else {
//                        println("SLSLSLSLSLSL")
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
                    } else {
//                        println("SRSSRSRSRSRSRSRS")
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
                    } else {
//                        println("IBIBIBIBIBIBIBIBI")
                    }
                    SmartDashboard.putBoolean("CamSLConnected", camSL?.isConnected ?: false)
                    SmartDashboard.putBoolean("CamSRConnected", camSR?.isConnected ?: false)
                    SmartDashboard.putBoolean("CamIBConnected", camIB?.isConnected ?: false)
                } catch (ex:Exception) {
                    println("Error in apriltag $ex")
                }
            }
        }
    }
}

private fun getEstimatedGlobalPose(camera: PhotonCamera, numTargets: Int, singleTagEstimator: PhotonPoseEstimator, multiTagEstimator: PhotonPoseEstimator? = null): Pose2d? {

    try {
        //Exclusion zones
        if (Drive.combinedPosition.x > 14.0 && Drive.combinedPosition.x < 39.0 && (Drive.combinedPosition.y < 17.0 || Drive.combinedPosition.y > 9.0)) {
            if (!OI.operatorController.start) {
                println("Don't trust AprilTag from here")
                return null
            }
        }

        if (!camera.isConnected) {
//            println("Camera not connected")
            return null
        }

        val cameraResult: MultiTargetPNPResult? = camera.latestResult.multiTagResult

        val validTargets = camera.latestResult.targets//.filter{ validTags.contains(it.fiducialId) && it.poseAmbiguity < maxAmbiguity }
//        println("WHSlkfjasdhflkjadhsflaefaAAAAAAAAAAAAAAAAAAA ${validTargets}")

        if (validTargets != null) {
            if (numTargets < 1) {
//                println("Apriltag: Not enough tags")
                return null
            }
        } else {
            println("Valid targets null")
            return null
        }

//        println("outside for loop")
        for (target in validTargets) {
            if (target.fiducialId > 16 || excludedTagsList.contains(target.fiducialId)) {
                println("AprilTag: Invalid Tag")
                return null
            }
        }
//        if ((validTargets.count() < 2 && cameraResult.estimatedPose.ambiguity > 0.05) || cameraResult.estimatedPose.ambiguity > 0.15) {
//            return null
//        }

        //println("at least 2 valid targets found ${poseList}")
//        println("WHAT THE HECK ${camera.name} : $numTargets")
        if  (multiTagEstimator != null && numTargets > 1) {
            if (cameraResult != null) {
                multiTagEstimator.setReferencePose(
                    Pose2d(
                        cameraResult.estimatedPose.best.translation.toTranslation2d(),
                        cameraResult.estimatedPose.best.rotation.toRotation2d()
                    )
                )
            }
        } else {
//            println("Using singletag on camera ${camera.name}!!! HI!")
            if (cameraResult != null) {
                singleTagEstimator.setReferencePose(
                    Pose2d(
                        cameraResult.estimatedPose.best.translation.toTranslation2d(),
                        cameraResult.estimatedPose.best.rotation.toRotation2d()
                    )
                )
            }
        }

        val newPose = if (multiTagEstimator != null && numTargets > 1) multiTagEstimator.update() else singleTagEstimator.update()

//                println("newPose: $newPose")
        if (newPose?.isPresent == true) {
            val result = newPose.get()
            //TODO: Min distance for multiTag
                for (target in validTargets) {
                    val dist = cameraResult?.estimatedPose?.best?.let {
                        Vector2(
                            it.x, cameraResult.estimatedPose.best.y)
                    }?.let {
                        Vector2(aprilTagFieldLayout.getTagPose(target.fiducialId).get().x, aprilTagFieldLayout.getTagPose(target.fiducialId).get().y).distance(
                            it
                        )
                    }


                    if (dist != null) {
                        if (dist > singleTagMinDist) {
//                            println("Apriltag: Too far!")
                            return null
                        }
                    }
                }
//                println("AprilTag: Single target too far away ${result.estimatedPose.toPose2d().toTMMField().y.absoluteValue} vs ${(FieldManager.chargeFromCenterY + FieldManager.chargingStationDepth).asFeet}")
//                    return null
//                }
//            println("Valid target found ${validTargets.count()}")
            var avgDist = 0.0.feet

            if (cameraResult != null) {
                for (target in validTargets)
                    avgDist += (Vector2(aprilTagFieldLayout.getTagPose(target.fiducialId).get().x, aprilTagFieldLayout.getTagPose(target.fiducialId).get().y).distance(Vector2(result.estimatedPose.toPose2d().x, result.estimatedPose.toPose2d().y)) / numTargets).meters
                    if (cameraResult != null) {
//                        println(
//                            "askldfhadlkfhasdlkfhasdlkfjhadslkfhasdlkjfasklghalkghsf: ${
//                                (Vector2(
//                                    aprilTagFieldLayout.getTagPose(
//                                        target.fiducialId
//                                    ).get().x, aprilTagFieldLayout.getTagPose(target.fiducialId).get().y
//                                ).distance(
//                                    Vector2(
//                                        cameraResult.estimatedPose.best.x,
//                                        cameraResult.estimatedPose.best.y
//                                    )
//                                ) / numTargets).meters
//                            }"
//                        )
                    }

                when (camera.name) {
                    "CamSL" -> {
                        lastSLDetectionTime = result.timestampSeconds; lastSLAmbiguity =
                            cameraResult.estimatedPose.ambiguity; lastSLDist = avgDist
                    }

                    "CamSR" -> {
                        lastSRDetectionTime = result.timestampSeconds; lastSRAmbiguity =
                            cameraResult.estimatedPose.ambiguity; lastSRDist = avgDist
                    }

                    "CamIB" -> {
                        lastIBDetectionTime = result.timestampSeconds; lastIBAmbiguity =
                            cameraResult.estimatedPose.ambiguity; lastIBDist = avgDist
                    }
                }
            }


            return result.estimatedPose.toPose2d()
        } else {
//            println("noPose")
            return null
        }
    } catch (ex: Exception) {
        println("***********************************************************AprilTag Failed. Try Resetting IT!!!!*****************************************************")
        return null
    }
}

fun resetCameras() {
    if (camSL == null || multiTagSLPoseEstimator == null) {
        try {
            if (pvTable.containsSubTable("CamSL")) {
                camSL = PhotonCamera("CamSL")
                multiTagSLPoseEstimator = PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camSL,
                    robotToCamSL
                )
                singleTagSLPoseEstimator = PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
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
    if (camSR == null || multiTagSRPoseEstimator == null) {
        try {
            if (pvTable.containsSubTable("CamSR")) {
                camSR = PhotonCamera("CamSR")
                multiTagSRPoseEstimator = PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camSR,
                    robotToCamSR
                )
                singleTagSRPoseEstimator = PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
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
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
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

fun get2DSpeakerOffset(): Pair<Length, Angle>? {
    try {
        if (camSL == null && camSR == null) {
            return null
        } else {
            if (!camSL!!.isConnected && !camSR!!.isConnected) {
                return null
            }
            var slDist = 0.0.feet
            var srDist = 0.0.feet
            var slRot = 0.0.degrees
            var srRot = 0.0.degrees
            var numCams = 0.0
            if (camSL != null) {
                if (camSL!!.isConnected) {
                    numCams += 1.0
                    val validSLTargets = camSL!!.latestResult.targets
                    for (target in validSLTargets) {
                        if (target.fiducialId == if (isRedAlliance) 4 else 7) {
                            //                    println("pitch SL: ${-robotToCamSL.rotation.y.radians}")
                            slDist =
                                (speakerTagHeight - robotToCamSL.z.meters) / tan(-robotToCamSL.rotation.y.radians + target.pitch.degrees)
                            //                    println("slDist: ${slDist}")
                            slRot = (10.0.degrees) + target.yaw.degrees
                        }
                    }
                }

            }
            if (camSL != null) {
                if (camSL!!.isConnected) {
                    numCams += 1.0
                    val validSRTargets = camSR!!.latestResult.targets
                    for (target in validSRTargets) {
                        if (target.fiducialId == if (isRedAlliance) 4 else 7) {
                            //                    println("pitch SR: ${-robotToCamSR.rotation.y.radians}")
                            srDist =
                                (speakerTagHeight - robotToCamSR.z.meters) / tan(-robotToCamSR.rotation.y.radians + target.pitch.degrees)
                            srRot = ((-10.0).degrees) + target.yaw.degrees
                            //                    println("sr =dist ${srDist}")
                        }
                    }
                }
            }

            return Pair((slDist.asFeet.meters + srDist.asFeet.meters) / numCams, slRot + srRot / numCams)
        }
    } catch (e: Exception) {
        return null
    }
}



data class AprilDetection (
    val timestamp: Double,
    val pose: Pose2d,
    val averageDistance: Length,
    val ambiguity: Double
)