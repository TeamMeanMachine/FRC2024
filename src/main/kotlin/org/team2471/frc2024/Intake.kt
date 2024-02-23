package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer
import org.team2471.frc2024.Robot.isCompBot

object Intake: Subsystem("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")

    private val intakingEntry = table.getEntry("Intaking")
    private val holdingNoteEntry = table.getEntry("Holding Note")

    private val bottomBreakEntry = table.getEntry("Bottom Break")
    private val topBreakEntry = table.getEntry("Top Break")

    private val intakeCurrentEntry = table.getEntry("Intake Current")

    private val feederCurrentEntry = table.getEntry("Feeder Current")

    val intakeMotorTop = MotorController(FalconID(Falcons.INTAKE_TOP))
    val intakeMotorBottom = MotorController(FalconID(Falcons.INTAKE_BOTTOM))
    val feederMotor = MotorController(FalconID(Falcons.FEEDER))

    private val bottomBreakSensor = DigitalInput(DigitalSensors.BOTTOM_BREAK)
    private val topBreakSensor = DigitalInput(DigitalSensors.TOP_BREAK)


    var intakeState = IntakeState.EMPTY


    val bottomBreak: Boolean
        get() = !bottomBreakSensor.get()
    val topBreak: Boolean
        get() = !topBreakSensor.get()

    var holdingCargo = false

    init {
        intakingEntry.setBoolean(false)
        holdingNoteEntry.setBoolean(true)

        var x = feederCurrentEntry.getDouble(0.0)

        intakeMotorTop.config {
            currentLimit(35, 60, 1)
            coastMode()
            inverted(isCompBot)
            followersInverted(isCompBot)
        }

        intakeMotorBottom.config {
            currentLimit(35, 60, 1)
            coastMode()
            inverted(isCompBot)
            followersInverted(isCompBot)
        }

        feederMotor.config {
            currentLimit(35, 40, 1)
            coastMode()
            inverted(false)
        }

        GlobalScope.launch {
            val t = Timer()
            t.start()
            var stagedT= 0.0

            periodic {
//                colorEntry.setString(colorSensor.color.toHexString())
//                proximityEntry.setInteger(colorSensor.proximity.toLong())

                intakeCurrentEntry.setDouble(intakeMotorTop.current)
                feederCurrentEntry.setDouble(feederMotor.current)
                intakeCurrentEntry.setDouble(intakeMotorTop.current)
                feederCurrentEntry.setDouble(feederMotor.current)
                bottomBreakEntry.setBoolean(bottomBreak)
                topBreakEntry.setBoolean(topBreak)

            }
        }

    }

    override suspend fun default() {
        val t = Timer()
        periodic {
            when(intakeState) {
                IntakeState.EMPTY -> {}
                IntakeState.SPITTING -> {}
                IntakeState.INTAKING -> {
                    Pivot.angleSetpoint = 18.0.degrees
                    setIntakeMotorsPercent(0.9)
                    if (bottomBreak) {
                        intakeState = IntakeState.SLOWING
                        t.start()
                    }
                }
                IntakeState.SLOWING -> {
                    setIntakeMotorsPercent(0.2)
                    if (topBreak) {
                        intakeState = IntakeState.REVERSING
                    }
                    if (t.get() > 2.0) {
                        intakeState = IntakeState.INTAKING
                    }
                }
                IntakeState.REVERSING -> {
                    setIntakeMotorsPercent(-0.1)
                    if (!topBreak) {
                        intakeState = IntakeState.HOLDING
                    }
                }
                IntakeState.HOLDING -> {
                    setIntakeMotorsPercent(0.0)
                }
            }
//            if (intaking) {
//                if (!detectedCargo && !holdingCargo) {
//                    Pivot.angleSetpoint = 18.0.degrees
//                    setIntakeMotorsPercent(0.9)
//                }
//                if (bottomBreak && !detectedCargo) {
//                    println("detected piece, slowing intake")
//                    setIntakeMotorsPercent(0.2)
//                    detectedCargo = true
//                }
//                if (topBreak && !holdingCargo) {
//                    setIntakeMotorsPercent(0.0)
//                    println("stopping intake")
//                    holdingCargo = true
//                    detectedCargo = false
//
//
////                    Pivot.angleSetpoint = Pivot.TESTPOSE
//                }
//                if (holdingCargo && !detectedCargo) {
//
//                }
//                if (detectedCargo) {
//                    if (t.get() > 2.0) {
//                        detectedCargo = false
//                    }
//                } else {
//                    t.start()
//                }
//            } else {
//                setIntakeMotorsPercent(0.0)
//                t.start()
//            }
        }
    }

    fun setIntakeMotorsPercent(value: Double) {
        intakeMotorTop.setPercentOutput(value)
        intakeMotorBottom.setPercentOutput(value)
        feederMotor.setPercentOutput(value)
    }

    enum class IntakeState() {
        EMPTY,
        INTAKING,
        SLOWING,
        REVERSING,
        HOLDING,
        SPITTING,
    }
}