package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem

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

    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP))

    val motorRpmBottom
        get() = shooterMotorBottom.velocity

    var shootingRPM = false
        set(value) {
            println("Shooter $value")
            if (value) {
                // AMP SHOT!!!!!!!!!!!!!!!!!!!!! Bottom: 12 Top: 15!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                rpmTop = shootingRpmTopEntry.getDouble(70.0)
                rpmBottom = shootingRpmBottomEntry.getDouble(70.0)
            } else {
                rpmTop = 0.0
                rpmBottom = 0.0
            }
            field = value
        }

    val motorRpmTop
        get() = shooterMotorTop.velocity

    val shootingRpmTop: Double
        get() = shootingRpmTopEntry.getDouble(20.0)

    val shootingRpmBottom: Double
        get() = shootingRpmBottomEntry.getDouble(21.0)

    var kFeedForwardTop = 60.0 / 6380.0
    var kFeedForwardBottom = 69.0 / 6380.0
/*    var rpm: Double = 0.0
        set(value) {
            println("setting rpm to $rpm")
            shooterMotorTop.setVelocitySetpoint(value, value * kFeedForwardTop) //value, value * kFeedForward)
            shooterMotorBottom.setVelocitySetpoint(value, value * kFeedForwardBottom) //value, value * kFeedForward)
            field = value
        }*/
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
        shooterPercentEntry.setDouble(1.0)
        if (!shootingRpmTopEntry.exists()) {
            shootingRpmTopEntry.setDouble(75.0)
            shootingRpmTopEntry.setPersistent()
        }

        if (!shootingRpmBottomEntry.exists()) {
            shootingRpmBottomEntry.setDouble(60.0)
            shootingRpmBottomEntry.setPersistent()
        }

        shooterMotorBottom.config {
            //add pid later, right now, p makes it go
            pid {
                p(0.0000015)
            }
            currentLimit(30, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        shooterMotorTop.config {
            pid {
                p(0.000015)
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
                rpmTopEntry.setDouble(rpmTop)
                rpmBottomEntry.setDouble(rpmBottom)

//                if (topDEntry.getDouble(1.0) != shooterMotorTop.getD() || topPEntry.getDouble(1.0) != shooterMotorTop.getP() || bottomDEntry.getDouble(1.0) != shooterMotorBottom.getD() ||bottomPEntry.getDouble(1.0) != shooterMotorBottom.getP()) {
//                    shooterMotorTop.setP(topPEntry.getDouble(1.0))
//                    shooterMotorTop.setD(topDEntry.getDouble(1.0))
//                    shooterMotorBottom.setD(bottomDEntry.getDouble(1.0))
//                    shooterMotorBottom.setP(bottomPEntry.getDouble(1.0))
//                    println("Top P: ${shooterMotorTop.getP()}  Top D: ${shooterMotorTop.getD()}  Bottom P: ${shooterMotorBottom.getP()}  Bottom D:  ${shooterMotorBottom.getD()}")
//                }

            }
        }
    }

    override suspend fun default() {
        periodic {
//            rpmTop = 0.0
//            rpmBottom = 0.0
        }
    }
}