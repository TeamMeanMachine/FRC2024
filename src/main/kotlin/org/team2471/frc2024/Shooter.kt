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

    private val shooterMotors = MotorController(FalconID(Falcons.SHOOTER_BOTTOM), FalconID(Falcons.SHOOTER_TOP))

    init {
        shooterPercentEntry.setDouble(1.0)

        shooterMotors.config {
            // Copied from bunny. Prolly way off
            currentLimit(20, 30, 1)
            coastMode()
            inverted(false)
            followersInverted(true)
        }

        GlobalScope.launch {
            periodic {
                shooterCurrentEntry.setDouble(shooterMotors.current)
            }
        }
    }
}