package org.team2471.frc2024

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

    fun reset() = navx.reset()

    fun isConnected() = navx.isConnected

    var deltaTime: Double = 1.0
        set(value) {
            field = value
        }
}