package org.team2471.bunnybots2023

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI

class NavxWrapper {
    private val navx = AHRS(SPI.Port.kMXP)
    val angle: Double
        get() = navx.yaw.toDouble()
    val roll: Double
        get() = navx.roll.toDouble()
    val pitch: Double
        get() = navx.pitch.toDouble()
    val rate: Double
        get() = navx.rawGyroZ.toDouble()

//    fun calibrate() = Unit

    fun reset() = navx.reset()

    var deltaTime: Double = 1.0
        set(value) {
            field = value
        }
}