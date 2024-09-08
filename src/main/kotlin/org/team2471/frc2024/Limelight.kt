//package org.team2471.frc2024
//import edu.wpi.first.math.filter.Debouncer
//import edu.wpi.first.math.geometry.*
//import edu.wpi.first.networktables.NetworkTable
//import kotlinx.coroutines.GlobalScope
//import kotlinx.coroutines.launch
//import org.team2471.frc.lib.coroutines.periodic
//import org.team2471.frc.lib.framework.Subsystem
//import org.team2471.frc.lib.math.Vector2L
//import org.team2471.frc.lib.math.setAdvantagePose
//import org.team2471.frc.lib.math.setAdvantagePoses
//import org.team2471.frc.lib.motion_profiling.MotionCurve
//import org.team2471.frc.lib.units.*
//import org.team2471.frc2024.Limelight.limelightDistCurve
//import org.team2471.frc.lib.vision.Camera
//import org.team2471.frc.lib.vision.GlobalPose
//import org.team2471.frc.lib.vision.LimelightHelpers
//
//object Limelight {
//
//
//    val limelight: LimelightCamera = LimelightCamera(AprilTag.aprilTable, "limelight-shooter")
//
//    val isConnected: Boolean
//        get() = limelight.isConnected
//
//    val limelightDistCurve = MotionCurve()
//
//
//    init {
//        rebuildCurves()
//
//        GlobalScope.launch {
//            periodic {
//                try {
//                    if (limelight.isConnected) limelight.getEstimatedGlobalPose()
//                } catch (ex: Exception) {
//                    println("Error in Limelight: $ex")
//                }
////                limelight.isConnectedEntry.setBoolean(limelight.isConnected)
//
//            }
//        }
//    }
//
//    fun getCurrentGlobalPose(ignoreHeartbeat: Boolean): GlobalPose? {
//        val lastGlobalPose = if (limelight.newHeartbeat || ignoreHeartbeat) limelight.lastGlobalPose else null
//        if (!ignoreHeartbeat) println("Not ignoring")
//        if (lastGlobalPose != null) {
////            limelight.lastGlobalPose = null
//        }
//        return lastGlobalPose
//    }
//
//    fun rebuildCurves() {
//        limelightDistCurve.setMarkBeginOrEndKeysToZeroSlope(false)
//
//        // dist in meters
//        limelightDistCurve.storeValue(1.5, 0.00075)
//        limelightDistCurve.storeValue(1.9, 0.00125)
//        limelightDistCurve.storeValue(2.5, 0.003)
//        limelightDistCurve.storeValue(3.0, 0.0065)
//        limelightDistCurve.storeValue(3.5, 0.0023)
//        limelightDistCurve.storeValue(4.0, 0.014)
//        limelightDistCurve.storeValue(4.5, 0.025) //photonvision says not to trust after 15 feet  old: 0.0165
//        limelightDistCurve.storeValue(5.0, 0.03) // 0.02
//        limelightDistCurve.storeValue(6.0, 0.04) // 0.03
//    }
//
//}
//
//class LimelightCamera(
//    networkTable: NetworkTable,
//    name: String,
//): GenericCamera(networkTable, name) {
//    val robotToCamera = Transform3d(
//        Translation3d(-0.185.meters.asMeters, -0.06.inches.asMeters, 0.15.inches.asMeters),
//        Rotation3d(0.0, 30.degrees.asRadians, -178.degrees.asRadians)
//    )
//
//    var previousHeartbeat = 0.0
//    override var isConnected: Boolean = false
//    override fun reset() {
//        println("limelight reset not implemented")
//    }
//
//    var newHeartbeat = false
//    var connectedDebouncer = Debouncer(0.1, Debouncer.DebounceType.kFalling)
//
//
//    init {
//
//        GlobalScope.launch {
//            periodic {
//                val tempHeartbeat = LimelightHelpers.getLimelightNTDouble(name,"hb")
//                newHeartbeat = tempHeartbeat != previousHeartbeat
//                isConnected = connectedDebouncer.calculate(newHeartbeat)
//                previousHeartbeat = tempHeartbeat
//
//
//
//            }
//        }
//    }
//
//
//
//
//
//    override fun getEstimatedGlobalPose(): GlobalPose? {
//        val poseArray = LimelightHelpers.getBotPoseEstimate_wpiBlue(name)   // LimelightHelpers.getBotPose3d_wpiBlue(name)
//        var estimatedPose = Vector2L(poseArray.pose.translation.x.meters, poseArray.pose.translation.y.meters * (8.2 / 8.0)) // * 1.017 to compensate for limelight being off on y
//        var tagCount = poseArray.tagCount
//        var targetPoses : ArrayList<Vector2L> = arrayListOf()
//
//        if (estimatedPose == Vector2L(0.0.meters, 0.0.meters) || tagCount < 2) {
//            targetPoseEntry.setAdvantagePoses(targetPoses.toTypedArray())
//            advantagePoseEntry.setAdvantagePoses(arrayOf(), arrayOf())
//            return null
//        } // estimatedPose returns origin if no tags seen
//
//        var avgDist = 0.0.inches
//        var avgAmbiguity = 0.0
//
//        for (target in poseArray.rawFiducials!!) {
//            target?.let{
//                avgDist += it.distToCamera.meters
//                avgAmbiguity += it.ambiguity
//
//            }
//        }
////            val visionTargetPosition = LimelightHelpers.getTargetPose_RobotSpace(name)
////            targetPoses.add(Vector2L(visionTargetPosition[0].meters, visionTargetPosition[1].meters))
//
// //       targetPoseEntry.setAdvantagePoses(targetPoses.toTypedArray())
//        avgDist /= poseArray.tagCount.toDouble()
//        avgAmbiguity /= poseArray.tagCount.toDouble()
//
//        var stDev = limelightDistCurve.getValue(avgDist.asMeters)
//
//        lastGlobalPose = GlobalPose(estimatedPose, Drive.heading, stDev, Timer.FPGATimestamp)
//
//        stDevEntry.setDouble(stDev)
//
//        advantagePoseEntry.setAdvantagePoses(arrayOf(estimatedPose), arrayOf(poseArray.pose.rotation.radians.radians))
//        // Todo filter more
//
//        return lastGlobalPose
//    }
//}