package org.team2471.frc2024

import com.revrobotics.ColorSensorV3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycle
import edu.wpi.first.wpilibj.I2C
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.util.Timer

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

    val intakeMotorTop = MotorController(FalconID(Falcons.INTAKE_TOP))
    val intakeMotorBottom = MotorController(FalconID(Falcons.INTAKE_BOTTOM))
    val feederMotor = MotorController(FalconID(Falcons.FEEDER))

    private val colorSensorI2CPort: I2C.Port = I2C.Port.kMXP
    private val colorSensor = ColorSensorV3(colorSensorI2CPort)
    private val button = DigitalInput(DigitalSensors.BUTTON)

    private var staged = false
    private var staging = false
    var intaking = false
        set(value) {
            println("intaking set to $value")
            field = value
//            if (staging || staged) return
            intakeMotorTop.setPercentOutput(if (value) 0.7 else 0.0)
            intakeMotorBottom.setPercentOutput(if (value) 0.7 else 0.0)
            feederMotor.setPercentOutput(if (value) 0.6 else 0.0)

        }

    private val proximity: Int
        get() = colorSensor.proximity

    init {
        intakingEntry.setBoolean(false)
        holdingNoteEntry.setBoolean(true)

        proximityThresholdEntry.setDouble(500.0)

        intakePercentEntry.setDouble(0.8)
        feederPercentEntry.setDouble(0.8)
        var x = feederCurrentEntry.getDouble(0.0)

        intakeMotorTop.config {
            // Copied from bunny. Prolly way off
            currentLimit(35, 40, 1)
            coastMode()
            inverted(false)
            followersInverted(false)
        }

        intakeMotorBottom.config {
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
            val t = Timer()
            t.start()
            var stagedT= 0.0

            periodic {
                colorEntry.setString(colorSensor.color.toHexString())
                proximityEntry.setInteger(colorSensor.proximity.toLong())

                intakeCurrentEntry.setDouble(intakeMotorTop.current)
                feederCurrentEntry.setDouble(feederMotor.current)
                colorEntry.setDouble(0.0)
                intakeCurrentEntry.setDouble(intakeMotorTop.current)
                feederCurrentEntry.setDouble(feederMotor.current)


            }
        }

    }

    override suspend fun default() {
        val t = Timer()
        periodic {
            //println("button ${button.get()}")
            if (intaking) {
//                if (!button.get() && (!staging || !staged)) {
//                    staging = true
//                    intakeMotors.setPercentOutput(0.3)
//                    feederMotor.setPercentOutput(0.3)
//                }
                if (proximity > 500) {
                    intakeMotorTop.setPercentOutput(0.0)
                    intakeMotorBottom.setPercentOutput(0.0)
                    feederMotor.setPercentOutput(0.0)
                    staged = true
                    staging = false
                }
//                if (t.get() > 0.14) {
//                    intakeMotors.setPercentOutput(0.0)
//                    feederMotor.setPercentOutput(0.0)
//                } else if (proximity > 135) {
//                    intakeMotors.setPercentOutput(0.3)
//                    feederMotor.setPercentOutput(0.3)
//                } else {
//                    t.start()
//                }
            } else {
                t.start()
            }
        }
    }

    suspend fun feedSetPower(power: Double) {
        feederMotor.setPercentOutput(power)
        delay(0.1)
        if (staged) staged = false
    }
}