package org.team2471.frc2024/*
package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

object Limelight : Subsystem("Limelight") {
    val intakeNT = NetworkTableInstance.getDefault().getTable("limelight-intake")
    val shootNT  = NetworkTableInstance.getDefault().getTable("limelight-shoot")
    val nt = NetworkTableInstance.getDefault().getTable("Limelights")

    private val intakeLimelightModeEntry : NetworkTableEntry = intakeNT.getEntry("pipeline")

    private val intakeSeesTargetsEntry : NetworkTableEntry = intakeNT.getEntry("tv")
    private val shootSeesTargetsEntry : NetworkTableEntry = shootNT.getEntry("tv")

    // output stuff
    private val latencyEntryIntake : NetworkTableEntry = nt.getEntry("Intake Latency")
    private val latencyEntryShoot : NetworkTableEntry = nt.getEntry("Shoot Latency")
    private val limelightModeEntry : NetworkTableEntry = nt.getEntry("Intake Limelight Mode")

    var intakeLimelightMode : LimelightPipeline = LimelightPipeline.NOTE
        set(value) { intakeLimelightModeEntry.setNumber(value.mode) }

    var pose : Vector2? = null
    var rotation : Double? = null
    var confidence : Double? = null
    var latency : Double? = null

    val seesNotes : Boolean
        get() = seesNotes()

    init {

        intakeLimelightMode = LimelightPipeline.NOTE

        GlobalScope.launch(MeanlibDispatcher) {

*/
/*            periodic {
                if (seesApriltags()) {
                    val botposeData : DoubleArray = limelightNetworkTable.getEntry("botpose_wpired").getDoubleArray(DoubleArray(0))
                    print(botposeData)
                    val totalLatency : Double = botposeData[6]
                    totalLatencyEntry.setDouble(totalLatency)
                    val fieldPose : Vector2 = wpiRedToMMCoords(Vector2(botposeData[0], botposeData[1]))
                    val rotation : Double = botposeData[5]

                    var intakeAprilTagsSeen = 0
                    var shootAprilTagsSeen = 0

                    for (entryNum in 0..7) {
                        if (intakeNT.getEntry("ta${entryNum}").getDouble(0.0) == 0.0) {
                            intakeAprilTagsSeen += 1
                        }
                    }

                    for (entryNum in 0 .. 7 ) {
                        if (shootNT.getEntry("ta${entryNum}").getDouble(0.0) == 0.0) {
                            shootAprilTagsSeen += 1
                        }
                    }

                    if (intakeAprilTagsSeen > shootAprilTagsSeen) {

                    } else {

                    }
                }

                if (intakeLimelightMode == LimelightPipeline.NOTE && seesNotes()) {

                }
            }
        }*//*

    }

    override suspend fun default() {

    }

    fun seesApriltags(): Boolean {
        return shootSeesTargetsEntry.getBoolean(false) || (intakeLimelightMode == LimelightPipeline.APRILTAG && intakeSeesTargetsEntry.getBoolean(false))
    }

    fun seesNotes(): Boolean {
        return intakeLimelightMode == LimelightPipeline.NOTE && intakeSeesTargetsEntry.getBoolean(false)
    }

    private fun wpiRedToMMCoords(position : Vector2) : Vector2 {
        return Vector2(-(position.y - 13.46875), position.x - 27.1354)
    }


    fun getNoteCoords(boundingBoxLowerY : Double, xDegreeOffset: Double): Vector2 {
        val angleFromBottom = boundingBoxLowerY/ NoteLimelight.limelightScreenHeight * NoteLimelight.limelightVerticalFOV
        val tangentAngle = (NoteLimelight.limelightVerticalFOV /2 - angleFromBottom) * Math.PI/180.0
        val distance = NoteLimelight.noteDistFromCenter + 1/12 * (sin(tangentAngle) + ((NoteLimelight.limelightHeight - (1- cos(tangentAngle)))/ tan(tangentAngle)))
        return NoteLimelight.limelightToRobotCoords(Vector2(distance / tan(xDegreeOffset), distance))
    }

    fun limelightToRobotCoords(position: Vector2): Vector2 {
        return position - limelightOffset
    }
}

enum class LimelightPipeline(val mode: Int) {
    APRILTAG(3),
    NOTE(0),
}*/
