package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

object NoteLimelight : Subsystem("NoteLimelight") {
    val limelightNetworkTable: NetworkTable = NetworkTableInstance.getDefault().getTable("limelight-intake")
    val limelightOffset = Vector2(0.0, 1.0)
    val noteDistFromCenter = 6.0
    val limelightHeight = 9.0
    val limelightVerticalFOV = 49.7
    val limelightScreenHeight = 320.0
    init {

    }

    override suspend fun default() {
        periodic {
            if (seesNotes()){
                for (entryNum in 0..7){
                    if (limelightNetworkTable.getEntry("ta${entryNum}").getDouble(0.0) == 0.0){

                    }
                }

            }
        }
    }
    fun limelightToRobotCoords(position: Vector2): Vector2 {
        return position-limelightOffset
    }
    fun seesNotes(): Boolean {
        return limelightNetworkTable.getEntry("tv").getBoolean(false)
    }
    fun getNoteRobotCoords(boundingBoxLowerY: Int, boundingBoxX: Int): Vector2 {
        val angleFromBottom = boundingBoxLowerY/limelightScreenHeight* limelightVerticalFOV
        val tangentAngle = (limelightVerticalFOV/2 - angleFromBottom) * Math.PI/180.0
        val distance = noteDistFromCenter + 1/12 * (sin(tangentAngle) + (1- cos(tangentAngle))/tan(tangentAngle))
        return limelightToRobotCoords(Vector2(boundingBoxX * distance, distance))
    }

}