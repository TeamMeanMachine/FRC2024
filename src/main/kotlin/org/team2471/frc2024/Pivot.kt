package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees

object Pivot: Subsystem("Pivot") {
    private val table = NetworkTableInstance.getDefault().getTable("Pivot")

    private val pivotCurrentEntry = table.getEntry("Pivot Current")

    private val ticksEntry = table.getEntry("Pivot Ticks")
    private val ticksOffsetEntry = table.getEntry("Pivot Tick Offset")
    private val angleEntry = table.getEntry("Pivot Angle")
    private val angleSetspointEntry = table.getEntry("Pivot Angle Setpoint")

    val pivotMotor = MotorController(FalconID(Falcons.PIVOT))

    private val pivotEncoder = AnalogInput(AnalogSensors.PIVOT)

    private val gearRatio = 1 / 61.71

    private const val MIN_HARDSTOP = 0.0

    private const val MAX_HARDSTOP = 111.0

    val pivotTicks: Int
        get() = pivotEncoder.value
    val pivotAngle: Angle
//                                                                                       Ticks to degrees ↓↓↓↓↓
           get() = (-pivotEncoder.value.degrees + ticksOffsetEntry.getDouble(3665.0).degrees) / 11.2

    var angleSetpoint: Angle = pivotAngle
        set(value) {
            var temp = value
            field = temp.asDegrees.coerceIn(MIN_HARDSTOP, MAX_HARDSTOP).degrees
            pivotMotor.setPositionSetpoint(field.asDegrees)
        }

    init {
        ticksOffsetEntry.setDouble(3665.0)

        pivotMotor.config() {
            //                              ticks / gear ratio
            feedbackCoefficient = (360.0 / 2048.0 / gearRatio)
//            brakeMode()
            coastMode()
            inverted(true)

            currentLimit(30, 40, 20)
        }

        GlobalScope.launch {
            periodic {
                pivotMotor.setRawOffset(pivotAngle.asDegrees)

                pivotCurrentEntry.setDouble(pivotMotor.current)
                ticksEntry.setDouble(pivotTicks.toDouble())
                angleEntry.setDouble(pivotAngle.asDegrees)
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