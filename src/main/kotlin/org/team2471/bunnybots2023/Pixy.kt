package org.team2471.bunnybots2023

import io.github.pseudoresonance.pixy2api.Pixy2
import io.github.pseudoresonance.pixy2api.Pixy2CCC
import io.github.pseudoresonance.pixy2api.links.SPILink

import edu.wpi.first.networktables.NetworkTableInstance
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block
import org.team2471.frc.lib.framework.Subsystem
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic

object Pixy : Subsystem("Pixy") {

    val table = NetworkTableInstance.getDefault().getTable("Pixy")
    val fpsEntry = table.getEntry("Pixy FPS")

    lateinit var pixy: Pixy2

    const val maxBlockCount : Int = 20
    const val screenHeight : Int = 255
    const val screenWidth : Int = 255

    init {
        initializePixy()
    }

    override suspend fun default() {
        periodic {
            if (pixy != null) {
//                println("pixy FPS: ${ pixy.fps }")
                val lowest = lowestBall()
//                println("x = ${lowest?.x} y = ${lowest?.y}" )
                fpsEntry.setDouble(pixy.fps.toDouble())
            }
        }
    }
    fun initializePixy() {
        pixy = Pixy2.createInstance(SPILink())
        pixy.init()
        pixy.setLamp(0.toByte(), 0.toByte())
    }

    fun lowestBall() : Block? { // returns x and y pos of lowest ball (closest ball) in pixels
        try {
            val errorCode: Int = pixy.ccc.getBlocks(false, (Pixy2CCC.CCC_SIG1.toInt()), maxBlockCount)
            if (errorCode != 0) {
                return null
            }

            return pixy.ccc.blockCache.maxByOrNull {it.y}

        } catch (ex:java.lang.Exception) {
            println("Pixy 2 exception $ex")
            return null
        }
    }
}




