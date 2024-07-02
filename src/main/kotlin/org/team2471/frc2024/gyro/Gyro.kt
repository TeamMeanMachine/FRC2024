package org.team2471.frc2024.gyro

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.AngularAcceleration
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

    val angle: Angle get() = if (isConnected) inputs.angle.degrees else headingFromSwerve()

    val roll: Angle get() = inputs.roll.degrees

    val pitch: Angle get() = inputs.pitch.degrees

    val rate: Double get() = if (isConnected) inputs.rate else headingVelocityFromSwerve() //degrees per second

    init {
        GlobalScope.launch {
            periodic(0.02) {
                io.updateInputs(inputs)
                Logger.processInputs("Gyro", inputs)
            }
        }
    }

    fun reset() = io.reset()

    fun headingFromSwerve(): Angle {
        //somehow calculate robot angle from swerve odometry

        val foo = Drive.modules[0].modulePosition - Drive.modules[3].modulePosition

//        println("angle ${foo.angle.asDegrees.round(2)} vector $foo")





        return foo.angle
    }

    fun headingVelocityFromSwerve(): Double {

        return 0.0
    }
}