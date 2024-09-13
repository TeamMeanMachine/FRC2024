package org.team2471.frc2024.gyro

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.RobotMode
import org.team2471.frc.lib.util.robotMode

object Gyro  {
    private val io: GyroIO = when (robotMode) {
        RobotMode.REAL -> NavxWrapper()
        RobotMode.SIM, RobotMode.REPLAY -> object: GyroIO {} //sim or replay do not use gyro
    }
    private val inputs = GyroIO.GyroIOInputs()

    val isConnected: Boolean get() = inputs.isConnected

    val angle: Angle get() = inputs.angle.degrees

    val roll: Angle get() = inputs.roll.degrees

    val pitch: Angle get() = inputs.pitch.degrees

    val rate: Double get() = inputs.rate //degrees per second

    init {
        GlobalScope.launch {
            periodic(0.02) {
                io.updateInputs(inputs)
                Logger.processInputs("Sensors/Gyro", inputs)
            }
        }
    }

    fun reset() = io.reset()
}