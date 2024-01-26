package org.team2471.frc2024

import com.revrobotics.ColorSensorV3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.I2C
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem

object Intake: Subsystem("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")

    private val intakingEntry = table.getEntry("Intaking")
    private val holdingNoteEntry = table.getEntry("Holding Note")

    private val colorEntry = table.getEntry("ColorSensor Color")
    private val proximityEntry = table.getEntry("ColorSensor Proximity")
    private val proximityThresholdEntry = table.getEntry("ColorSensor Proximity Threshold")

    private val intakePercentEntry = table.getEntry("Intake Percent")
    private val intakeCurrentEntry = table.getEntry("Intake Current")

    private val feederPercentEntry = table.getEntry("Feeder Percent")
    private val feederCurrentEntry = table.getEntry("Feeder Current")

    private val intakeMotors = MotorController(FalconID(Falcons.INTAKE_LOWER), FalconID(Falcons.INTAKE_UPPER))
    private val feederMotor = MotorController(FalconID(Falcons.FEEDER))

    private val colorSensorI2CPort: I2C.Port = I2C.Port.kMXP
    private val colorSensor = ColorSensorV3(colorSensorI2CPort)

    val proximity: Int
        get() = colorSensor.proximity

    init {
        intakingEntry.setBoolean(false)
        holdingNoteEntry.setBoolean(true)

        proximityThresholdEntry.setDouble(500.0)

        intakePercentEntry.setDouble(0.7)
        feederPercentEntry.setDouble(0.7)

        intakeMotors.config {
            // Copied from bunny. Prolly way off
            currentLimit(20, 30, 1)
            coastMode()
            inverted(false)
            followersInverted(true)
        }

        feederMotor.config {
            // Also copied from bunny. Still prolly way off
            currentLimit(20, 30, 1)
            coastMode()
            inverted(false)
        }

        GlobalScope.launch {
            periodic {
                colorEntry.setString(colorSensor.color.toHexString())
                proximityEntry.setInteger(colorSensor.proximity.toLong())

                if (proximity > proximityThresholdEntry.getDouble(500.0)) {
                    intakeMotors.setPercentOutput(0.0)
                    feederMotor.setPercentOutput(0.0)
                }
            }
        }

    }

}