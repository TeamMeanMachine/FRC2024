package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import java.util.Vector
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

object NoteLimelight : Subsystem("NoteLimelight") {
    val limelightNetworkTable: NetworkTable = NetworkTableInstance.getDefault().getTable("limelight-intake")
    val nt : NetworkTable = NetworkTableInstance.getDefault().getTable("Intake Limelight")
    val totalLatencyEntry = nt.getEntry("Total Latency")
    val limelightOffset = Vector2(0.0, 1.0)
    val noteDistFromCenter = 6.0
    val limelightHeight = 9.0 // change when LL put on robot; height to camera
    val limelightVerticalFOV = 49.7
    val limelightScreenHeight = 320.0
    val limelightScreenWidth = 320.0

    var notes : ArrayList<Vector2> = arrayListOf()

    override suspend fun default() {
        periodic {

            val totalLatency = limelightNetworkTable.getEntry("tl").getDouble(0.0) + limelightNetworkTable.getEntry("cl").getDouble(0.0)
            totalLatencyEntry.setDouble(totalLatency);

            if (seesNotes()){
                notes = ArrayList()
                for (entryNum in 0..7){
                    if (limelightNetworkTable.getEntry("ta${entryNum}").getDouble(0.0) == 0.0){
                        val xOffset : Double = limelightNetworkTable.getEntry("tx${entryNum}").getDouble(0.0)
                        val boundingBoxY = limelightNetworkTable.getEntry("ty${entryNum}").getDouble(0.0)/limelightVerticalFOV * limelightScreenHeight + limelightScreenHeight/2
                        val lowerY = boundingBoxY - limelightNetworkTable.getEntry("tvert${entryNum}").getDouble(0.0)/2
                        val noteCoords : Vector2 = getNoteCoords(lowerY, xOffset)

                        notes.add(noteCoords)

                    }
                }
            }
        }
    }
    fun limelightToRobotCoords(position: Vector2): Vector2 {
        return position - limelightOffset
    }
    fun seesNotes(): Boolean {
        return limelightNetworkTable.getEntry("tv").getBoolean(false)
    }

    fun getNoteCoords(boundingBoxLowerY : Double, xDegreeOffset: Double): Vector2 {
        val angleFromBottom = boundingBoxLowerY/limelightScreenHeight* limelightVerticalFOV
        val tangentAngle = (limelightVerticalFOV/2 - angleFromBottom) * Math.PI/180.0
        val distance = noteDistFromCenter + 1/12 * (sin(tangentAngle) + ((limelightHeight - (1- cos(tangentAngle)))/tan(tangentAngle)))
        return limelightToRobotCoords(Vector2(distance/tan(xDegreeOffset), distance))
    }

}