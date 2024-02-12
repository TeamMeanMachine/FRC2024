package org.team2471.frc2024

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.util.Timer
import kotlin.math.absoluteValue

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

suspend fun fire() = use(Shooter, Intake){
    Shooter.rpm = Shooter.shootingRpm
    val t = Timer()
    t.start()
    periodic {
        if (((Shooter.rpmTop + Shooter.rpmBottom) - (Shooter.rpm * 2)).absoluteValue < 10.0) {
            println("at rpm shooting now!!!")
            this.stop()
        }
        if (t.get() > 2.0) {
            println("waited 1.0 seconds shooting at lower power")
            this.stop()
        }
    }
    Intake.intakeMotors.setPercentOutput(0.5)
    Intake.feederMotor.setPercentOutput(1.0)
    t.start()
    periodic {
        if (t.get() > 1.0) {
            println("exiting shooting")
            this.stop()
        }
    }
    Intake.intaking = false
//    Shooter.shooting = false
}