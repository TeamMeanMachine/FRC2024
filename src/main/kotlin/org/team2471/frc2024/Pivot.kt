package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.Robot.isCompBot

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

    const val TESTPOSE = 32

    // All in degrees
    val CLOSESPEAKERPOSE = 62
//        get() = stageAngleEntry.getDouble(60.0)

    const val MINHARDSTOP = 5.5

    const val MAXHARDSTOP = 110.2

    // Ticks
    private val MINTICKS = if (isCompBot) 2540.0 else 323.0
    private val MAXTICKS = if (isCompBot) 1410.0 else 238.0

    val pivotTicks: Int
        get() = pivotEncoder.value

    val encoderVoltage: Double
        get() = pivotEncoder.voltage

    val pivotEncoderAngle: Angle
        get() = linearMap(MINTICKS, MAXTICKS, MINHARDSTOP, MAXHARDSTOP, pivotEncoder.value.toDouble()).degrees

    val pivotMotorAngle: Angle
        get() = pivotMotor.position.degrees

    var angleSetpoint: Angle = pivotEncoderAngle
        set(value) {
            field = value.asDegrees.coerceIn(MINHARDSTOP, MAXHARDSTOP).degrees
            pivotMotor.setPositionSetpoint(field.asDegrees)
            println("Setpoint changed to: $field")
        }




    init {
//        ticksOffsetEntry.setDouble(3665.0)
        stageAngleEntry.setDouble(20.0)

        pivotMotor.config {
            pid {
                p(0.0002)
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



//                pivotMotor.setRawOffset(pivotEncoderAngle.asDegrees)
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