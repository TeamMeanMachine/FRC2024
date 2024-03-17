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

    private val manualIntake = table.getEntry("Manual Intake")

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

    val holdingCargo: Boolean
        get() = intakeState != IntakeState.EMPTY && intakeState != IntakeState.INTAKING

    init {
        intakingEntry.setBoolean(false)
        holdingNoteEntry.setBoolean(true)
        manualIntake.setBoolean(false)

        var x = feederCurrentEntry.getDouble(0.0)

        intakeMotorTop.config {
            currentLimit(35, 60, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        intakeMotorBottom.config {
            currentLimit(35, 60, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        feederMotor.config {
            currentLimit(38, 70, 1)
            coastMode()
            inverted(false)
        }

        GlobalScope.launch {
            val t = Timer()
            t.start()
            var stagedT= 0.0

            periodic {
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
                IntakeState.EMPTY -> {
                    setIntakeMotorsPercent(0.0)
                }
                IntakeState.SPITTING -> {}
                IntakeState.INTAKING -> {
                    Pivot.angleSetpoint = 18.0.degrees
                    setIntakeMotorsPercent(0.9)
                    OI.driverController.rumble = 0.7
                    if (bottomBreak) {
                        intakeState = IntakeState.SLOWING
                        t.start()
                    }
                }
                IntakeState.SLOWING -> {
                    if (manualIntake.getBoolean(false)) {
                        feederMotor.setPercentOutput(0.25)
                    } else {
                        setIntakeMotorsPercent(0.25)
                    }
                    if (topBreak) {
                        intakeState = IntakeState.REVERSING
                    }
                    if (t.get() > 2.0) {
                        intakeState = IntakeState.INTAKING
                    }
                }
                IntakeState.REVERSING -> {
                    if (manualIntake.getBoolean(false)) {
                        feederMotor.setPercentOutput(-0.1)
                    } else {
                        setIntakeMotorsPercent(-0.1)
                    }
                    if (!topBreak) {
                        intakeState = IntakeState.HOLDING
                    }
                }
                IntakeState.HOLDING -> {
                    if (manualIntake.getBoolean(false)) {
                        feederMotor.setPercentOutput(0.0)
                    } else {
                        setIntakeMotorsPercent(0.0)
                    }
                }
                IntakeState.SHOOTING -> {
                    setIntakeMotorsPercent(1.0)
                }
            }
        }
    }

    fun setIntakeMotorsPercent(value: Double) {
        intakeMotorTop.setPercentOutput(value)
        intakeMotorBottom.setPercentOutput(value)
        feederMotor.setPercentOutput(value)
    }

    enum class IntakeState {
        EMPTY,
        INTAKING,
        SLOWING,
        REVERSING,
        HOLDING,
        SPITTING,
        SHOOTING
    }
}