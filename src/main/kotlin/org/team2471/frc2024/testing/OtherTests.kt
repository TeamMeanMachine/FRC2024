package org.team2471.frc2024.testing

import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.input.whenTrue
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.*


suspend fun Intake.motorsTest() {
    var switch = 0
    var intakeP = 0.0
    var feedP = 0.6
    var shootP = 0.0 //0.2
    var shoot2 = 0.15
    periodic {
        if (OI.operatorController.dPad == Controller.Direction.UP) {
            when (switch) {
                0 -> intakeP += 0.1
                1 -> feedP += 0.1
                2 -> shootP += 0.1
            }
        }
        if (OI.operatorController.dPad == Controller.Direction.DOWN) {
            when (switch) {
                0 -> intakeP -= 0.1
                1 -> feedP -= 0.1
                2 -> shootP -= 0.1
            }
        }

        OI.operatorController::a.whenTrue {
            switch += 1
            if (switch > 2) switch = 0
        }
        intakeMotorTop.setPercentOutput(intakeP)
        intakeMotorBottom.setPercentOutput(intakeP)
        feederMotor.setPercentOutput(feedP)
//        Shooter.shooterMotorBottom.setPercentOutput(shootP)
        shoot2 = shootP //if (shootP - .05 > 0) shootP - 0.05 else shootP
        Shooter.shooterMotorTop.setPercentOutput(shoot2)
        println("switch: $switch     intakeP: $intakeP     feedP: $feedP        shootP: $shootP")
    }
}

suspend fun Climb.motorTest() {
    periodic {
        climberMotor.setPercentOutput(OI.operatorLeftY)
//        OI.operatorController::a.whenTrue { relayOn = !relayOn }
//        println("relayOn: $relayOn")
    }
}

suspend fun Pivot.motorTest() {
    pivotMotor.setPercentOutput(0.2)
    delay(0.5)
    pivotMotor.setPercentOutput(0.0)
}

suspend fun Pivot.feedForwardTest() {
    var power = 0.0
    val startingAngle = pivotEncoderAngle
    periodic {
        pivotMotor.setPercentOutput(power)
        println("power: $power pivot: ${pivotEncoderAngle.asDegrees} current ${pivotMotor.current}")
        if (pivotEncoderAngle - startingAngle > 0.5.degrees) {
            println("power is now $power")
            this.stop()
        }
        power += 0.0001
    }
}
suspend fun Shooter.rpmTest() = use(Shooter) {
//    var switch = 0
//    var intakeP = 0.0
//    var feedP = 0.6
    var shootP = 5000.0 //0.2
    var upPressed = false
    var downPressed = false
//    var shoot2 = 0.15
    periodic {
        if (OI.operatorController.dPad == Controller.Direction.UP) {
            upPressed = true
        } else if (OI.operatorController.dPad == Controller.Direction.DOWN) {
            downPressed = true
        }
        if (OI.operatorController.dPad != Controller.Direction.UP && upPressed) {
            upPressed = false
            shootP += 100.0
        }
        if (OI.operatorController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            shootP -= 100.0
        }
        rpmTop = shootP
        rpmBottom = shootP

//        Shooter.shooterMotorTop.setPercentOutput(shootP)
        println("rpmBottom: ${shooterMotorBottom.velocity.round(1)}, rpmTop: ${shooterMotorTop.velocity.round(1)}, Set Point ${shootP.round(1)}")
    }
}