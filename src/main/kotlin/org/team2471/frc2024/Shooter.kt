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

    val shooterMotorOne = MotorController(FalconID(Falcons.SHOOTER_BOTTOM))
    val shooterMotorTwo = MotorController(FalconID(Falcons.SHOOTER_TOP))

    init {
        shooterPercentEntry.setDouble(1.0)

        shooterMotorOne.config {
            // Copied from bunny. Prolly way off
            currentLimit(35, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        shooterMotorOne.config {
            // Copied from bunny. Prolly way off
            currentLimit(35, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        GlobalScope.launch {
            periodic {
                shooterCurrentEntry.setDouble(shooterMotorOne.current)
            }
        }
    }

    override suspend fun default() {
        periodic {
            shooterCurrentEntry.setDouble(shooterMotorOne.current)
        }
    }
}