package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
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

    var shootingRPM = false
        set(value) {
            println("Shooter $value")
            if (value) {
                // AMP SHOT!!!!!!!!!!!!!!!!!!!!! Bottom: 12 Top: 14!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pivot Angle: 107.5
                // STAGE SHOT!!!!! Bottom 80: Top: 80   Pivot Angle: 32
                rpmTop = shootingRpmTopEntry.getDouble(70.0)
                rpmBottom = shootingRpmBottomEntry.getDouble(70.0)
            } else {
                rpmTop = 0.0
                rpmBottom = 0.0
            }
            field = value
        }

    val shootingRpmTop: Double
        get() = shootingRpmTopEntry.getDouble(20.0)

    val shootingRpmBottom: Double
        get() = shootingRpmBottomEntry.getDouble(21.0)

    var kFeedForwardTop = 70.0 / 6380.0
    var kFeedForwardBottom = 70.0 / 6380.0

    val pitchCurve = MotionCurve()
    val RPMCurve = MotionCurve()

    var rpmTop: Double = 0.0
        set(value) {
//            println("setting top rpm $value")
            shooterMotorTop.setVelocitySetpoint(value, value * kFeedForwardTop)
            field = value
        }
    var rpmBottom: Double = 0.0
        set(value) {
//            println("setting bottom rpm $value  feedForward ${(value * kFeedForwardBottom).round(4)}")
            shooterMotorBottom.setVelocitySetpoint(value, value * kFeedForwardBottom)
            field = value
        }

    init {

        if (!Pitch3Entry.exists()) {
            Pitch3Entry.setDouble(-8.0)
            Pitch6Entry.setDouble(-19.0)
            Pitch9Entry.setDouble(-27.0)
            Pitch15Entry.setDouble(-30.0)
            Pitch21Entry.setDouble(24.8)
            RPM3Entry.setDouble(2800.0)
            RPM6Entry.setDouble(3300.0)
            RPM9Entry.setDouble(3950.0)
            RPM15Entry.setDouble(5200.0)
            RPM21Entry.setDouble(3200.0)

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

                if (Robot.isEnabled && (motorRpmTop - rpmTop).absoluteValue + (motorRpmBottom - rpmBottom).absoluteValue < 500.0 && rpmTop + rpmBottom > 20.0) {
                    OI.driverController.rumble = 0.8
                } else {
                    OI.driverController.rumble = 0.0
                }

//                println("entry: ${RPM3Entry.getDouble(5.0)}   curve: ${RPMCurve.getValue(3.0)}")
                if (Pitch3Entry.getDouble(3.0)!=pitchCurve.getValue(3.0)) { rebuildCurves() }
                if (Pitch6Entry.getDouble(6.0)!=pitchCurve.getValue(6.0)) { rebuildCurves() }
                if (Pitch9Entry.getDouble(9.0)!=pitchCurve.getValue(9.0)) { rebuildCurves() }
                if (Pitch15Entry.getDouble(15.0)!=pitchCurve.getValue(15.0)) { rebuildCurves() }
                if (Pitch21Entry.getDouble(21.0)!=pitchCurve.getValue(21.0)) { rebuildCurves() }
                if (RPM3Entry.getDouble(5.0)!=RPMCurve.getValue(3.0)) { rebuildCurves() }
                if (RPM6Entry.getDouble(6.0)!=RPMCurve.getValue(6.0)) { rebuildCurves() }
                if (RPM9Entry.getDouble(9.0)!=RPMCurve.getValue(9.0)) { rebuildCurves() }
                if (RPM15Entry.getDouble(15.0)!=RPMCurve.getValue(15.0)) { rebuildCurves() }
                if (RPM21Entry.getDouble(21.0)!=RPMCurve.getValue(21.0)) { rebuildCurves() }
            }
        }
    }

    override suspend fun default() {
        periodic {
            rpmTopEntry.setDouble(rpmTop)
            rpmBottomEntry.setDouble(rpmBottom)
        }
    }

    fun rebuildCurves() {
        println("Rebuilding curves. Hi.")
        pitchCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        pitchCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        pitchCurve.storeValue(3.0, Pitch3Entry.getDouble(8.0))
        pitchCurve.storeValue(6.0, Pitch6Entry.getDouble(19.0))
        pitchCurve.storeValue(9.0, Pitch9Entry.getDouble(27.0))
        pitchCurve.storeValue(15.0, Pitch15Entry.getDouble(30.0))
        pitchCurve.storeValue(21.0, Pitch21Entry.getDouble(32.0))

        RPMCurve.setMarkBeginOrEndKeysToZeroSlope(false)
        RPMCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        RPMCurve.storeValue(3.0, RPM3Entry.getDouble(2800.0))
        RPMCurve.storeValue(6.0, RPM6Entry.getDouble(3300.0))
        RPMCurve.storeValue(9.0, RPM9Entry.getDouble(3950.0))
        RPMCurve.storeValue(15.0, RPM15Entry.getDouble(5200.0))
        RPMCurve.storeValue(21.0, RPM21Entry.getDouble(5650.0))

    }

}