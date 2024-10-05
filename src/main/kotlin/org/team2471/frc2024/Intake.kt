package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer

object Intake: Subsystem("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")

    private val intakingEntry = table.getEntry("Intaking")
    private val holdingNoteEntry = table.getEntry("Holding Note")
    private val manualIntake = table.getEntry("Manual Intake")
    private val bottomBreakEntry = table.getEntry("Bottom Break")
    private val topBreakEntry = table.getEntry("Top Break")
    private val intakeCurrentEntry = table.getEntry("Intake Current")
    private val feederCurrentEntry = table.getEntry("Feeder Current")

    val intakeMotorTop = MotorController(FalconID(Falcons.INTAKE_TOP, "Intake/Top"))
    val intakeMotorBottom = MotorController(FalconID(Falcons.INTAKE_BOTTOM, "Intake/Bottom"))
    val feederMotor = MotorController(FalconID(Falcons.FEEDER, "Intake/Feeder"))

    private val bottomBreakSensor = DigitalInput(DigitalSensors.BOTTOM_BREAK)
    private val topBreakSensor = DigitalInput(DigitalSensors.TOP_BREAK)


    var intakeState = IntakeState.EMPTY


    val bottomBreak: Boolean
        get() = !bottomBreakSensor.get()
    val topBreak: Boolean
        get() = !topBreakSensor.get()

    val holdingCargo: Boolean
        get() = intakeState != IntakeState.EMPTY && intakeState != IntakeState.INTAKING && intakeState != IntakeState.SPITTING

    val intakeAngle = 43.0.degrees

    init {
        manualIntake.setBoolean(false)

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
            periodic {
                intakeCurrentEntry.setDouble(intakeMotorTop.current)
                feederCurrentEntry.setDouble(feederMotor.current)
                intakeCurrentEntry.setDouble(intakeMotorTop.current)
                feederCurrentEntry.setDouble(feederMotor.current)
                bottomBreakEntry.setBoolean(bottomBreak)
                topBreakEntry.setBoolean(topBreak)
                holdingNoteEntry.setBoolean(holdingCargo)
                intakingEntry.setBoolean(intakeState == IntakeState.INTAKING)

            }
        }

    }

    override suspend fun default() {
        val t = Timer()
        var bottomBreakCounter = 0
        periodic(0.01) {
            when(intakeState) {
                IntakeState.EMPTY -> {
                    setIntakeMotorsPercent(0.0)
                }
                IntakeState.SPITTING -> {
                    Intake.intakeMotorTop.setPercentOutput(-0.9)
                    Intake.intakeMotorBottom.setPercentOutput(-0.9)
                    Intake.feederMotor.setPercentOutput(-0.9)
                }
                IntakeState.INTAKING -> {
                    Pivot.angleSetpoint = intakeAngle
                    Shooter.manualShootState = false
                    Shooter.setRpms(0.0)
                    setIntakeMotorsPercent(0.9)
                    OI.driverController.rumble = 0.7
                    if (bottomBreak && bottomBreakCounter > 1) {
                        intakeState = IntakeState.SLOWING
                        t.start()
                    } else if (bottomBreak) {
                        bottomBreakCounter ++
                    } else {
                        bottomBreakCounter = 0
                    }
                }
                IntakeState.SLOWING -> {
                    Pivot.angleSetpoint = intakeAngle
                    if (manualIntake.getBoolean(false)) {
                        feederMotor.setPercentOutput(0.1)
                    } else {
                        setIntakeMotorsPercent(0.1)
                    }
                    if (topBreak || !bottomBreak) {
                        intakeState = IntakeState.REVERSING
                    }
                    if (t.get() > 2.0) {
                        intakeState = IntakeState.INTAKING
                    }
                    Shooter.manualShootState = false
                }
                IntakeState.REVERSING -> {
                    Pivot.angleSetpoint = Pivot.DRIVEPOSE
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

    override fun preEnable() {
        GlobalScope.launch {
            println("inside intake preEnable ${Robot.recentTimeTaken()}")
        }
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