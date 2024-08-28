package org.team2471.frc2024.gyro

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
    class GyroIOInputs: LoggableInputs {
        var isConnected: Boolean = false
        var angle: Double = 0.0
        var roll: Double = 0.0
        var pitch: Double = 0.0
        var rate: Double = 0.0

        override fun toLog(table: LogTable) {
            table.put("isConnected", isConnected)
            table.put("Angle", angle)
            table.put("Roll", roll)
            table.put("Pitch", pitch)
            table.put("Rate", rate)
        }

        override fun fromLog(table: LogTable) {
            isConnected = table.get("isConnected", isConnected)
            angle = table.get("Angle", angle)
            roll = table.get("Roll", roll)
            pitch = table.get("Pitch", pitch)
            rate = table.get("Rate", rate)
        }
    }

    fun updateInputs(inputs: GyroIOInputs) {}
    fun reset() {}
}