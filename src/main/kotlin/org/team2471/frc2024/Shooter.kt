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
    private val rpmOneEntry = table.getEntry("RPM One")
    private val rpmTwoEntry = table.getEntry("RPM Two")

    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP))
    var rpmOne = 0.0
        get() = shooterMotorBottom.velocity

    var rpmTwo = 0.0
        get() = shooterMotorTop.velocity
    init {
        shooterPercentEntry.setDouble(1.0)

        shooterMotorBottom.config {
            // Copied from bunny. Prolly way off
            currentLimit(30, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        shooterMotorTop.config {
            // Copied from bunny. Prolly way off
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
        }
    }
}