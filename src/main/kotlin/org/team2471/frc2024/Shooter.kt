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
    private val rpmOneEntry = table.getEntry("RPM Bottom")
    private val rpmTwoEntry = table.getEntry("RPM Top")
    private val rpmEntry = table.getEntry("rpm Setpoint")
    private val shootingRpmEntry = table.getEntry("Shooting RPM")

    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP))

    val rpmOne
        get() = shooterMotorBottom.velocity

    val rpmTwo
        get() = shooterMotorTop.velocity

    val shootingRpm: Double
        get() = shootingRpmEntry.getDouble(20.0)

    var rpmTop = 0.0
        set(value) {
            field = value
            rpm = rpm
        }
    var rpmBottom = 0.0
        set(value) {
            field = value
            rpm = rpm
        }

    var kFeedForward = 70.0 / 6380.0
    var rpm: Double = 0.0
        set(value) {
//            println("setting rpm to $rpm")
            shooterMotorTop.setVelocitySetpoint(0.0, 0.0) //value, value * kFeedForward)
            shooterMotorBottom.setVelocitySetpoint(0.0, 0.0) //value, value * kFeedForward)
            field = value
        }
    init {
        shooterPercentEntry.setDouble(1.0)
        if (!shootingRpmEntry.exists()) {
            shootingRpmEntry.setDouble(20.0)
            shootingRpmEntry.setPersistent()
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

        GlobalScope.launch {
            periodic {
                shooterCurrentEntry.setDouble(shooterMotorBottom.current)
//                if (rpmOne > 20.0) println("rpmOne = $rpmOne rpmTwo = $rpmTwo")
                shooterCurrentEntry.setDouble(shooterMotorBottom.current)
                shooterTwoCurrentEntry.setDouble(shooterMotorTop.current)
                rpmOneEntry.setDouble(rpmOne)
                rpmTwoEntry.setDouble(rpmTwo)
                rpmEntry.setDouble(rpm)
            }
        }
    }

    override suspend fun default() {
        periodic {
            rpm = 0.0
        }
    }
}