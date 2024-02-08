package org.team2471.frc2024

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.units.inches

suspend fun climbWithTrigger() = use(Climb) {
    println("inside climbWIthTrigger!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    periodic {
        if (OI.operatorRightTrigger < 0.02) {
            this.stop()
        }
        Climb.climbSetpoint = (OI.operatorLeftTrigger * (Climb.MAX_CLIMB_INCHES - Climb.MIN_CLIMB_INCHES) + Climb.MIN_CLIMB_INCHES).inches
        println("climbing with ${OI.operatorController.rightTrigger} setpoint ${Climb.climbSetpoint}")
    }
}