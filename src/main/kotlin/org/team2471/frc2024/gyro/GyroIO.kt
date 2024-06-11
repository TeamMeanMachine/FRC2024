package org.team2471.frc2024.gyro

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team9432.lib.advantagekit.kGet
import org.team9432.lib.advantagekit.kPut

interface GyroIO {
    class GyroIOInputs: LoggableInputs {
        var isConnected: Boolean = false
        var angle: Double = 0.0
        var roll: Double = 0.0
        var pitch: Double = 0.0
        var rate: Double = 0.0

        override fun toLog(table: LogTable) {
            table.kPut("isConnected", isConnected)
            table.kPut("Angle", angle)
            table.kPut("Roll", roll)
            table.kPut("Pitch", pitch)
            table.kPut("Rate", rate)
        }

        override fun fromLog(table: LogTable) {
            table.kGet("isConnected", isConnected)
            table.kGet("Angle", angle)
            table.kGet("Roll", roll)
            table.kGet("Pitch", pitch)
            table.kGet("Rate", rate)
        }
    }

    fun updateInputs(inputs: GyroIOInputs) {}
    fun reset() {}
}