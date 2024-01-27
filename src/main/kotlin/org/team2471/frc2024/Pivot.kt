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

    private val pivotMotor = MotorController(FalconID(Falcons.PIVOT))

    private val pivotEncoder = AnalogInput(AnalogSensors.PIVOT)

    private val gearRatio = 1 / 61.71

    val pivotTicks: Int
        get() = pivotEncoder.value
    val pivotAngle: Angle
//                                                                                   Ticks to degrees ↓↓↓↓↓
           get() = (pivotEncoder.value.degrees + ticksOffsetEntry.getDouble(0.0).degrees) / 11.2

    init {
        ticksOffsetEntry.setDouble(0.0)

        pivotMotor.config() {
            //                              ticks / gear ratio
            feedbackCoefficient = (360.0 / 2048.0 / gearRatio)

            brakeMode()
            inverted(false)

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

}