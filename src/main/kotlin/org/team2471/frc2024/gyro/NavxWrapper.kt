package org.team2471.frc2024.gyro

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI

class NavxWrapper: GyroIO {
    private val navx = AHRS(SPI.Port.kMXP)

    override fun reset() = navx.reset()

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.isConnected = navx.isConnected
        inputs.angle = navx.yaw.toDouble()
        inputs.pitch = navx.pitch.toDouble()
        inputs.roll = navx.roll.toDouble()
        inputs.rate = navx.rawGyroZ.toDouble()
    }
}