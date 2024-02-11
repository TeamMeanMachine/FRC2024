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
        Climb.climbSetpoint = (OI.operatorRightTrigger * (Climb.MAX_CLIMB_INCHES - Climb.MIN_CLIMB_INCHES) + Climb.MIN_CLIMB_INCHES).inches
        println("climbing with ${OI.operatorController.rightTrigger} setpoint ${Climb.climbSetpoint} tried to set to ${(OI.operatorRightTrigger * (Climb.MAX_CLIMB_INCHES - Climb.MIN_CLIMB_INCHES) + Climb.MIN_CLIMB_INCHES).inches}")
    }
}

suspend fun spit() = use(Intake) {
    println("starting spit periodic")
    periodic {
        if (OI.driverController.rightBumper) {
            Intake.intakeMotors.setPercentOutput(-0.9)
            Intake.feederMotor.setPercentOutput(-0.9)
        } else {
            this.stop()
        }
    }
}