package org.team2471.frc2024.gyro

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.RobotMode
import org.team2471.frc.lib.util.robotMode
import org.team2471.frc2024.Drive

object Gyro  {
    private val io: GyroIO = when (robotMode) {
        RobotMode.REAL -> NavxWrapper()
        RobotMode.SIM, RobotMode.REPLAY -> object: GyroIO {} //sim or replay do not use gyro
    }
    private val inputs = GyroIO.GyroIOInputs()

    val isConnected: Boolean get() = inputs.isConnected

    val angle: Angle get() = if (isConnected) inputs.angle.degrees else 0.0.degrees

    val roll: Angle get() = inputs.roll.degrees

    val pitch: Angle get() = inputs.pitch.degrees

    val rate: Double get() = if (isConnected) inputs.rate else headingVelocityFromSwerve() //degrees per second

    init {
        GlobalScope.launch {
            periodic(0.02) {
                io.updateInputs(inputs)
                Logger.processInputs("Sensors/Gyro", inputs)
            }
        }
    }

    fun reset() = io.reset()

    fun headingFromSwerve(): Angle {
        //somehow calculate robot angle from swerve odometry

        val foo = Drive.modules[0].fieldPosition - Drive.modules[3].fieldPosition
//        val fooTwo = Drive.modules[1].fieldPosition - Drive.modules[2].fieldPosition


        val angle = -foo.angle

//        println("angle ${angle.asDegrees.round(2)} vector $foo")
        return angle// + fooTwo.angle) / 2.0
    }

    fun headingVelocityFromSwerve(): Double {

        return 0.0
    }
}