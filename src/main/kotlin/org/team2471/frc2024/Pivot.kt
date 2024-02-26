package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.math.squareWithSign
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.units.Angle.Companion.cos
import org.team2471.frc2024.Drive.speakerPos
import org.team2471.frc2024.Robot.isCompBot
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sqrt

object Pivot: Subsystem("Pivot") {
    private val table = NetworkTableInstance.getDefault().getTable("Pivot")

    private val pivotCurrentEntry = table.getEntry("Pivot Current")

    private val ticksEntry = table.getEntry("Pivot Ticks")
    private val encoderAngleEntry = table.getEntry("Pivot Encoder Angle")
    private val motorAngleEntry = table.getEntry("Pivot Motor Angle")
    private val angleSetpointEntry = table.getEntry("Pivot Angle Setpoint")
    private val encoderVoltageEntry = table.getEntry("Encoder Voltage")
    private val stageAngleEntry = table.getEntry("Stage Angle")

    val pivotMotor = MotorController(FalconID(Falcons.PIVOT))

    private val pivotEncoder = AnalogInput(AnalogSensors.PIVOT)

    private val gearRatio = 1 / 61.71

    val TESTPOSE = 30.0.degrees //18 //32
    val CLOSESPEAKERPOSE = 62.0.degrees
    val MINHARDSTOP = 5.5.degrees
    val DRIVEPOSE = MINHARDSTOP + 2.0.degrees
    val MAXHARDSTOP = 110.2.degrees

    // Ticks
    private val MINTICKS = if (isCompBot) 2592.0 else 2124.0
    private val MAXTICKS = if (isCompBot) 1393.0 else 940.0


    var autoAim = false

    val pivotTicks: Int
        get() = pivotEncoder.value

    val encoderVoltage: Double
        get() = pivotEncoder.voltage

    val pivotEncoderAngle: Angle
        get() = linearMap(MINTICKS, MAXTICKS, MINHARDSTOP.asDegrees, MAXHARDSTOP.asDegrees, pivotEncoder.value.toDouble()).degrees

    val pivotMotorAngle: Angle
        get() = pivotMotor.position.degrees

    var angleSetpoint: Angle = pivotEncoderAngle
        set(value) {
            field = value.asDegrees.coerceIn(MINHARDSTOP.asDegrees, MAXHARDSTOP.asDegrees).degrees
            pivotMotor.setPositionSetpoint(angleSetpoint.asDegrees, 0.024 * (cos((pivotEncoderAngle + 20.0.degrees).asRadians)) /*+ 0.000001*/)

//            println("set pivot angle to $field")
        }

    val pivotError: Double
        get() = (pivotEncoderAngle - angleSetpoint).asDegrees.absoluteValue




    init {
//        ticksOffsetEntry.setDouble(3665.0)
        stageAngleEntry.setDouble(20.0)

        pivotMotor.config {
            pid {
                p(0.00016)
                d(0.000004)
            }

            //                              ticks / gear ratio   fudge factor
            feedbackCoefficient = (360.0 / 2048.0 / gearRatio) * (107.0 / 305.0)
//            brakeMode()
            coastMode()
            inverted(true)

            currentLimit(35, 40, 20)
        }

        pivotMotor.setRawOffset(pivotEncoderAngle.asDegrees)


        GlobalScope.launch {
            periodic {
                pivotCurrentEntry.setDouble(pivotMotor.current)
                ticksEntry.setDouble(pivotTicks.toDouble())
                encoderAngleEntry.setDouble(pivotEncoderAngle.asDegrees)
                motorAngleEntry.setDouble(pivotMotorAngle.asDegrees)
                encoderVoltageEntry.setDouble(encoderVoltage)
                angleSetpointEntry.setDouble(angleSetpoint.asDegrees)
//                pivotErrorEntry.setDouble(pivotError)

                pivotMotor.setRawOffset(pivotEncoderAngle.asDegrees)

//                if (pivotError > 0.25) {
//                    pivotMotor.setPositionSetpoint(angleSetpoint.asDegrees, 0.024 * (cos((pivotEncoderAngle + 20.0.degrees).asRadians)) /*+ 0.000001*/)
////                    println(0.025 * cos((pivotEncoderAngle - 20.0.degrees).asRadians))
//                }

                if (autoAim) {
                    val dist = PoseEstimator.currentPose.distance(speakerPos)

                    // Calculated. May change a lot with more data
//                    val angle = (90.0 * (0.751492.pow(dist))).degrees

                    val angle = Shooter.pitchCurve.getValue(dist).degrees
                    println("Angle: ${angle}")
                    angleSetpoint = angle
                }
            }
        }

    }

    override fun postEnable() {
        pivotMotor.brakeMode()
    }

    override fun onDisable() {
        pivotMotor.coastMode()
    }

}