package org.team2471.frc2024

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.units.*
import org.team2471.frc2024.Robot.beforeFirstEnable

object PoseEstimator {

    val poseTable = NetworkTableInstance.getDefault().getTable("Pose Estimator")

    private val advantagePoseEntry = poseTable.getEntry("Combined Advantage Pose")
    private val maAdvantagePoseEntry = poseTable.getEntry("MA Combined Advantage Pose")
    private val kAprilEntry = poseTable.getEntry("kApril")

    private val offsetEntry = poseTable.getEntry("Offset")
    private val lastResetEntry = poseTable.getEntry("LastResetTime")
    private val startingPosEntry = poseTable.getEntry("Starting Pose Check")
    private val startingHeadingEntry = poseTable.getEntry("Starting Heading Check")
    private val apriltagHeadingEntry = poseTable.getEntry("Apriltag Heading")
    private var offset = Vector2(0.0, 0.0)
    private var kAprilScalar: Double = 0.05
    var headingOffset = 0.0.degrees
    private var lastZeroTimestamp = 0.0
    val currentPose //in feet
        get() = (robotPosM - offset).times(3.280839895) //this number is meters to feet conversion
    var preEnableHadTarget = false

    val robotPosM
        get() = Vector2(Drive.position.x.feet.asMeters, Drive.position.y.feet.asMeters)
    // val heading
    //   get() = (Drive.heading - headingOffset).wrap()

    init {
        apriltagHeadingEntry.setDouble(0.0)
        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                startingHeadingEntry.setBoolean((isBlueAlliance && (Drive.heading > 179.0.degrees || Drive.heading < -179.0.degrees)) || (isRedAlliance && Drive.heading > -1.0.degrees && Drive.heading < 1.0.degrees))

//Todo fix this
//                startingPosEntry.setBoolean((FieldManager.startingPosition - Drive.combinedPosition).length < 0.25)
                //untested ^

                advantagePoseEntry.setDoubleArray(doubleArrayOf(currentPose.x.feet.asMeters,  currentPose.y.feet.asMeters, Drive.heading.asDegrees))
                offsetEntry.setDoubleArray(doubleArrayOf(offset.x, offset.y))
                //Todo: starting positions for autos
//                if (DriverStation.isDisabled() && beforeFirstEnable && !preEnableHadTarget && !Drive.demoMode){
//                    robotPosM = FieldManager.startingPosition
//                    Drive.heading = if (FieldManager.isBlueAlliance) 180.0.degrees else 0.0.degrees
//                }
//                val maPose = MAPoseEstimator.latestPose
//                maAdvantagePoseEntry.setDoubleArray(doubleArrayOf(maPose.x, maPose.y, maPose.rotation.degrees))

            }
        }
    }
    fun addVision(detection: AprilDetection, numTarget: Int) {
        //Ignoring Vision data if timestamp is before the last zero

        if (detection.timestamp < (lastZeroTimestamp + 0.3)) { // || robotPosM == Vector2(0.0,0.0)) {
            println("Ignoring update during reset") // and initialization ...")
            return
        } else {
            try {
                val kAprilFinal = (kAprilScalar * (1 - detection.ambiguity) * (if (numTarget == 1) 0.25 else numTarget / 2.0)).coerceIn(0.0, 1.0)
//                    println(detection.ambiguity)
//                val kHeading = if (kotlin.math.abs(currentPose.y) > 15.0) kHeadingEntry.getDouble(0.001) else 0.0
//                    val latencyPose = Drive.lookupPose(detection.timestamp)
                if (DriverStation.isDisabled() && /*latencyPose == null && */beforeFirstEnable) {
                    val apriltagPoseF = Vector2(detection.pose.x.meters.asFeet, detection.pose.y.meters.asFeet)
                    preEnableHadTarget = true
                    Drive.position = apriltagPoseF
                    Drive.heading = detection.pose.rotation.radians.radians
                }
//                    if (latencyPose != null) {
//                        val odomDiff = robotPosM - latencyPose.position
                val apriltagPose = Vector2(detection.pose.x, detection.pose.y)// + odomDiff
                //val apriltagHeading = (-(detection.pose.rotation.degrees.degrees + headingDiff)).wrap180()
                offset = offset * (1.0 - kAprilFinal) + (robotPosM - apriltagPose) * kAprilFinal
                //apriltagHeadingEntry.setDouble(apriltagHeading.asDegrees)
                //headingOffset = headingOffset * (1.0 - kHeading) + apriltagHeading.unWrap180(Drive.heading) * kHeading
                //TODO: figure out coercing
//                        val coercedOffsetX = offset.x.coerceIn(
//                            -FieldManager.fieldHalfInFeet.x + robotPosM.x,
//                            FieldManager.fieldHalfInFeet.x + robotPosM.x
//                        )
//                        val coercedOffsetY = offset.y.coerceIn(
//                            -FieldManager.fieldHalfInFeet.y + robotPosM.y,
//                            FieldManager.fieldHalfInFeet.y + robotPosM.y
//                        )
//                        val coercedOffset = Vector2(coercedOffsetX, coercedOffsetY)
//                        if (coercedOffset.distance(offset) > 0.0) {
//                            offset = coercedOffset
//                            DriverStation.reportWarning("PoseEstimator: Offset coerced onto field", false)
//                            println("PoseEstimator: Offset coerced onto field")
//                        }
                //println("Heading Offset: ${apriltagHeading.unWrap(Drive.heading)}")

                //        println(offset)
//                    }
            } catch (ex: Exception) {
            }
        }
    }
    fun zeroOffset() {
        lastZeroTimestamp = Timer.getFPGATimestamp()
        offset = Vector2(0.0, 0.0)
        lastResetEntry.setDouble(lastZeroTimestamp)
    }
}

val isRedAlliance: Boolean
    get() {
        if (DriverStation.getAlliance().isEmpty) {
//                println("DriverStation.getAlliance() = null!!!!!!!!!!!!!!!!!! defaulting to isRedAlliance to true")
            return true
        } else {
            return DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        }
    }
val isBlueAlliance: Boolean
    get() = !isRedAlliance