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

    val intakeMotors = MotorController(FalconID(Falcons.INTAKE_BOTTOM), FalconID(Falcons.INTAKE_TOP))
    val feederMotor = MotorController(FalconID(Falcons.FEEDER))

    private val colorSensorI2CPort: I2C.Port = I2C.Port.kMXP
    private val colorSensor = ColorSensorV3(colorSensorI2CPort)

    private var staged = false

    private val proximity: Int
        get() = colorSensor.proximity

    init {
        intakingEntry.setBoolean(false)
        holdingNoteEntry.setBoolean(true)

        proximityThresholdEntry.setDouble(500.0)

        intakePercentEntry.setDouble(0.7)
        feederPercentEntry.setDouble(0.7)

        intakeMotors.config {
            // Copied from bunny. Prolly way off
            currentLimit(35, 40, 1)
            coastMode()
            inverted(false)
            followersInverted(false)
        }

        feederMotor.config {
            // Also copied from bunny. Still prolly way off
            currentLimit(35, 40, 1)
            coastMode()
            inverted(false)
        }

        GlobalScope.launch {
            periodic {
                colorEntry.setString(colorSensor.color.toHexString())
                proximityEntry.setInteger(colorSensor.proximity.toLong())

                println("color: ${colorSensor.color}   prox: ${colorSensor.proximity}")
                intakeCurrentEntry.setDouble(intakeMotors.current)
                feederCurrentEntry.setDouble(feederMotor.current)

                if (proximity > 128) { //proximityThresholdEntry.getDouble(125.0)) {
                    intakeMotors.setPercentOutput(0.0)
                    feederMotor.setPercentOutput(0.0)
                    staged = true
                }
            }
        }

    }

    override suspend fun default() {
        periodic {
            colorEntry.setDouble(0.0)
        }
    }

    fun feedSetPower(power: Double) {
        feederMotor.setPercentOutput(power)
        if (staged) staged = false
    }
}