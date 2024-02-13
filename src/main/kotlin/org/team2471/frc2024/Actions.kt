package org.team2471.frc2024

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.util.Timer

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
            Intake.intakeMotorTop.setPercentOutput(-0.9)
            Intake.intakeMotorBottom.setPercentOutput(-0.9)
            Intake.feederMotor.setPercentOutput(-0.9)
        } else {
            this.stop()
        }
    }
}

suspend fun fire() = use(Shooter, Intake){
/*    Shooter.rpmTop = Shooter.shootingRpmTop
    Shooter.rpmBottom = Shooter.shootingRpmBottom*/
    val t = Timer()
/*    t.start()
    periodic {
        println("combined rpm error: ${(Shooter.motorRpmTop - Shooter.rpmTop).absoluteValue + (Shooter.motorRpmBottom - Shooter.rpmBottom).absoluteValue}")
        if ((Shooter.motorRpmTop - Shooter.rpmTop).absoluteValue + (Shooter.motorRpmBottom - Shooter.rpmBottom).absoluteValue < 4.0) {
            println("at rpm shooting now!!! took ${t.get().round(3)} seconds")
            this.stop()
        }

        if (t.get() > 2.0) {
            println("waited 1.0 seconds shooting at lower power")
            this.stop()
        }
    }*/
    Intake.intakeMotorTop.setPercentOutput(0.5)
    Intake.intakeMotorBottom.setPercentOutput(0.5)
    Intake.feederMotor.setPercentOutput(1.0)
    t.start()
    periodic {
        if (t.get() > 2.0) {
            println("exiting shooting")
            this.stop()
        }
    }
    Intake.intaking = false
//    Shooter.shooting = false
    Shooter.rpmTop = 0.0
    Shooter.rpmBottom = 0.0
}