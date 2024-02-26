package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.motion_profiling.MotionCurve
import kotlin.math.absoluteValue

object Shooter: Subsystem("Shooter") {
    private val table = NetworkTableInstance.getDefault().getTable("Shooter")

    private val shooterPercentEntry = table.getEntry("Shooter Percent")
    private val shooterCurrentEntry = table.getEntry("Shooter Current")
    private val shooterTwoCurrentEntry = table.getEntry("Shooter Two Current")
    private val motorRpmBottomEntry = table.getEntry("RPM Bottom")
    private val motorRpmTopEntry = table.getEntry("RPM Top")
    private val rpmTopEntry = table.getEntry("rpm top setpoint")
    private val rpmBottomEntry = table.getEntry("rpm bottom setpoint")
    private val shootingRpmTopEntry = table.getEntry("Shooting RPM Top")
    private val shootingRpmBottomEntry = table.getEntry("Shooting RPM Bottom")
    private val topPEntry = table.getEntry("top P")
    private val topDEntry = table.getEntry("top D")
    private val bottomDEntry = table.getEntry("bottom D")
    private val bottomPEntry = table.getEntry("bottom P")
    private val shootingEntry = table.getEntry("shooting")
    val Pitch3Entry = table.getEntry("Pitch3Entry")
    val Pitch6Entry = table.getEntry("Pitch6Entry")
    val Pitch9Entry = table.getEntry("Pitch9Entry")
    val Pitch15Entry = table.getEntry("Pitch15Entry")
    val Pitch21Entry = table.getEntry("Pitch21Entry")
    val RPM3Entry = table.getEntry("RPM3Entry")
    val RPM6Entry = table.getEntry("RPM6Entry")
    val RPM9Entry = table.getEntry("RPM9Entry")
    val RPM15Entry = table.getEntry("RPM15Entry")
    val RPM21Entry = table.getEntry("RPM21Entry")

    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP))

    val motorRpmTop
        get() = shooterMotorTop.velocity

    val motorRpmBottom
        get() = shooterMotorBottom.velocity

    var manualShootState = false
        set(value) {
            println("Shooter $value")
            if (value) {
                // AMP SHOT!!!!!!!!!!!!!!!!!!!!! Bottom: 12 Top: 14!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pivot Angle: 107.5
                // STAGE SHOT!!!!! Bottom 80: Top: 80   Pivot Angle: 32
                rpmTopSetpoint = shootingRpmTopEntry.getDouble(1000.0)
                rpmBottomSetpoint = shootingRpmBottomEntry.getDouble(1000.0)
            } else {
                rpmTopSetpoint = 0.0
                rpmBottomSetpoint = 0.0
            }
            field = value
        }

    val shootingRpmTop: Double
        get() = shootingRpmTopEntry.getDouble(20.0)

    val shootingRpmBottom: Double
        get() = shootingRpmBottomEntry.getDouble(21.0)

    var kFeedForwardTop = 1.0/5000.0//76.0 / 6380.0
    var kFeedForwardBottom = 1.0/5000.0//76.0 / 6380.0

    val pitchCurve = MotionCurve()
    val RPMCurve = MotionCurve()

    private val topPDController = PDController(0.00015, 0.0006)
    private val bottomPDController = PDController(0.00015, 0.0006)

    private var ffTopPower: Double = 0.0
    private var ffBottomPower: Double = 0.0

    private var pdPowerTop = 0.0
    private var pdPowerBottom = 0.0

    const val MAXRPM = 5300.0;
    var rpmTopSetpoint: Double = 0.0
        set(value) {
            ffTopPower = value * kFeedForwardTop
//            println("setting top rpm $value")
//            shooterMotorTop.setVelocitySetpoint(value.coerceIn(0.0, MAXRPM), value * kFeedForwardTop)
            field = value
        }
    var rpmBottomSetpoint: Double = 0.0
        set(value) {
            ffBottomPower = value * kFeedForwardBottom
//            println("setting bottom rpm $value  feedForward ${(value * kFeedForwardBottom).round(4)}")
//            shooterMotorBottom.setVelocitySetpoint(value.coerceIn(0.0, MAXRPM), value * kFeedForwardBottom)
            field = value
        }

    init {

        if (!Pitch3Entry.exists()) {
            Pitch3Entry.setDouble(59.0)
            Pitch6Entry.setDouble(50.0)
            Pitch9Entry.setDouble(37.5)
            Pitch15Entry.setDouble(30.5)
            Pitch21Entry.setDouble(27.6)
            RPM3Entry.setDouble(3500.0)
            RPM6Entry.setDouble(3750.0)
            RPM9Entry.setDouble(5000.0)
            RPM15Entry.setDouble(5000.0)
            RPM21Entry.setDouble(5000.0)

            Pitch3Entry.setPersistent()
            Pitch6Entry.setPersistent()
            Pitch9Entry.setPersistent()
            Pitch15Entry.setPersistent()
            Pitch21Entry.setPersistent()
            RPM3Entry.setPersistent()
            RPM6Entry.setPersistent()
            RPM9Entry.setPersistent()
            RPM15Entry.setPersistent()
            RPM21Entry.setPersistent()
        }

        shooterPercentEntry.setDouble(1.0)
        if (!shootingRpmTopEntry.exists()) {
            shootingRpmTopEntry.setDouble(5000.0)
            shootingRpmTopEntry.setPersistent()
        }

        if (!shootingRpmBottomEntry.exists()) {
            shootingRpmBottomEntry.setDouble(5000.0)
            shootingRpmBottomEntry.setPersistent()
        }

        shooterMotorBottom.config {
            feedbackCoefficient = 53.0 * (400.0 / 350.0)
            //add pid later, right now, p makes it go
            pid {
                p(0.00015)
            }
            currentLimit(30, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        shooterMotorTop.config {
            feedbackCoefficient = 53.0 * (400.0 / 350.0)
            pid {
                p(0.00015)
            }
            currentLimit(30, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }
        topDEntry.setDouble(shooterMotorTop.getD())
        topPEntry.setDouble(shooterMotorTop.getP())
        bottomPEntry.setDouble(shooterMotorBottom.getP())
        bottomDEntry.setDouble(shooterMotorBottom.getD())

        GlobalScope.launch {
            periodic {
                shooterCurrentEntry.setDouble(shooterMotorBottom.current)
//                if (rpmOne > 20.0) println("rpmOne = $rpmOne rpmTwo = $rpmTwo")
                shooterCurrentEntry.setDouble(shooterMotorBottom.current)
                shooterTwoCurrentEntry.setDouble(shooterMotorTop.current)
                motorRpmBottomEntry.setDouble(motorRpmBottom)
                motorRpmTopEntry.setDouble(motorRpmTop)
                shootingEntry.setBoolean(manualShootState)

                if (Robot.isEnabled && (motorRpmTop - rpmTopSetpoint).absoluteValue + (motorRpmBottom - rpmBottomSetpoint).absoluteValue < 500.0 && rpmTopSetpoint + rpmBottomSetpoint > 20.0) {
                    OI.driverController.rumble = 1.0
                    OI.operatorController.rumble = 0.6
                } else {
                    if (Robot.isEnabled && Intake.intakeState != Intake.IntakeState.EMPTY) {
                        OI.driverController.rumble = 1.0
                        OI.operatorController.rumble = 0.0
                    } else {
                        OI.driverController.rumble = 0.0
                        OI.operatorController.rumble = 0.0
                    }
                }


                if (Pivot.autoAim) {
                    rpmTopSetpoint = RPMCurve.getValue(Drive.distance)
                    rpmBottomSetpoint = RPMCurve.getValue(Drive.distance)
                }

//                println("entry: ${RPM3Entry.getDouble(5.0)}   curve: ${RPMCurve.getValue(3.0)}")
                if (Pitch3Entry.getDouble(3.0)!=pitchCurve.getValue(3.0)) { rebuildCurves() }
                if (Pitch6Entry.getDouble(6.0)!=pitchCurve.getValue(6.0)) { rebuildCurves() }
                if (Pitch9Entry.getDouble(9.0)!=pitchCurve.getValue(9.0)) { rebuildCurves() }
                if (Pitch15Entry.getDouble(13.7)!=pitchCurve.getValue(13.7)) { rebuildCurves() }
                if (Pitch21Entry.getDouble(21.0)!=pitchCurve.getValue(21.0)) { rebuildCurves() }
                if (RPM3Entry.getDouble(5.0)!=RPMCurve.getValue(3.0)) { rebuildCurves() }
                if (RPM6Entry.getDouble(6.0)!=RPMCurve.getValue(6.0)) { rebuildCurves() }
                if (RPM9Entry.getDouble(9.0)!=RPMCurve.getValue(9.0)) { rebuildCurves() }
                if (RPM15Entry.getDouble(13.7)!=RPMCurve.getValue(13.7)) { rebuildCurves() }
                if (RPM21Entry.getDouble(21.0)!=RPMCurve.getValue(21.0)) { rebuildCurves() }


                if (Robot.isEnabled) {
                    pdPowerTop += topPDController.update(rpmTopSetpoint - motorRpmTop)
                    pdPowerBottom += bottomPDController.update(rpmBottomSetpoint - motorRpmBottom)

                    if (pdPowerTop + ffTopPower > 1.0) {
                        pdPowerTop = 1.0 - ffTopPower
                    }
                    if (pdPowerBottom + ffBottomPower > 1.0) {
                        pdPowerBottom = 1.0 - ffBottomPower
                    }

                    shooterMotorTop.setPercentOutput(pdPowerTop + ffTopPower)
                    shooterMotorBottom.setPercentOutput(pdPowerBottom + ffBottomPower)

                    println("topPower: $ffTopPower   bottomPower: $ffBottomPower")
                }
            }
        }
    }

    override suspend fun default() {
        periodic {
            rpmTopEntry.setDouble(rpmTopSetpoint)
            rpmBottomEntry.setDouble(rpmBottomSetpoint)
        }
    }

    fun rebuildCurves() {
        println("Rebuilding curves. Hi.")
        pitchCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        pitchCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        pitchCurve.storeValue(3.0, Pitch3Entry.getDouble(61.0))
        pitchCurve.storeValue(6.0, Pitch6Entry.getDouble(50.0))
        pitchCurve.storeValue(9.0, Pitch9Entry.getDouble(42.0))
        pitchCurve.storeValue(13.7, Pitch15Entry.getDouble(30.0))
        pitchCurve.storeValue(21.0, Pitch21Entry.getDouble(27.0))

        RPMCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        RPMCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        RPMCurve.storeValue(3.0, RPM3Entry.getDouble(3500.0))
        RPMCurve.storeValue(6.0, RPM6Entry.getDouble(3750.0))
        RPMCurve.storeValue(9.0, RPM9Entry.getDouble(4000.0))
        RPMCurve.storeValue(13.7, RPM15Entry.getDouble(4500.0))
        RPMCurve.storeValue(21.0, RPM21Entry.getDouble(5000.0))

    }

}