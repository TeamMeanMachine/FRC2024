package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2

object AprilTagLimelight : Subsystem("ApriltagLimelight") {
    val limelightNetworkTable : NetworkTable = NetworkTableInstance.getDefault().getTable("limelight-shoot")
    val nt : NetworkTable = NetworkTableInstance.getDefault().getTable("Shoot Limelight")
    val totalLatencyEntry = nt.getEntry("Total Latency")

    init {

        GlobalScope.launch(MeanlibDispatcher) {

            periodic {
                if (seesApriltags()) {
                    val botposeData : DoubleArray = limelightNetworkTable.getEntry("botpose_wpired").getDoubleArray(DoubleArray(0))
                    print(botposeData)
                    val totalLatency : Double = botposeData[16]
                    totalLatencyEntry.setDouble(totalLatency)
                    val fieldPose : Vector2 = wpiRedToMMCoords(Vector2(botposeData[3], botposeData[7]))
                }
            }
        }
    }

    override suspend fun default() {

    }

    fun seesApriltags(): Boolean {
        return limelightNetworkTable.getEntry("tv").getBoolean(false)
    }

    private fun wpiRedToMMCoords(position : Vector2) : Vector2 {
        return Vector2(-(position.y - 13.46875), position.x - 27.1354)
    }
}