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

    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP))

    val rpmOne
        get() = shooterMotorBottom.velocity

    val rpmTwo
        get() = shooterMotorTop.velocity

    var kFeedForward = 20.0 / (6000.0 * (66.0 / 23.0))
    var rpm: Double = 0.0
        set(value) {
            println("setting rpm to $value")
            shooterMotorTop.setVelocitySetpoint(0.0, value * kFeedForward) //value, value * kFeedForward)
            shooterMotorBottom.setVelocitySetpoint(0.0, value * kFeedForward) //value, value * kFeedForward)
            field = 0.0 //value
        }
    init {
        shooterPercentEntry.setDouble(1.0)

        shooterMotorBottom.config {
            //add pid later, right now, p makes it go
            feedbackCoefficient = 1.0 //66.0 / 23.0
            currentLimit(30, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        shooterMotorTop.config {
            feedbackCoefficient = 1.0 //66.0 / 23.0
            currentLimit(30, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        GlobalScope.launch {
            periodic {
                shooterCurrentEntry.setDouble(shooterMotorBottom.current)
                if (rpmOne > 20.0) println("rpmOne = $rpmOne rpmTwo = $rpmTwo")
            }
        }
    }

    override suspend fun default() {
        periodic {
            shooterCurrentEntry.setDouble(shooterMotorBottom.current)
            shooterTwoCurrentEntry.setDouble(shooterMotorTop.current)
            rpmOneEntry.setDouble(rpmOne)
            rpmTwoEntry.setDouble(rpmTwo)
            rpmEntry.setDouble(rpm)
        }
    }
}