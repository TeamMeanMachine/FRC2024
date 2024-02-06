package org.team2471.frc2024.testing

import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.input.whenTrue
import org.team2471.frc2024.Climber
import org.team2471.frc2024.Intake
import org.team2471.frc2024.OI
import org.team2471.frc2024.Shooter


suspend fun Intake.motorsTest() {
    var switch = 0
    var intakeP = 0.3
    var feedP = 0.4
    var shootP = 0.9 //0.2
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
        intakeMotors.setPercentOutput(intakeP)
        feederMotor.setPercentOutput(feedP)
        Shooter.shooterMotorOne.setPercentOutput(shootP)
        shoot2 = shootP //if (shootP - .05 > 0) shootP - 0.05 else shootP
        Shooter.shooterMotorTwo.setPercentOutput(shoot2)
        println("switch: $switch     intakeP: $intakeP     feedP: $feedP        shootP: $shootP")
    }
}

suspend fun Climber.motorTest() {
    periodic {
        climberMotor.setPercentOutput(0.6 * OI.operatorLeftY)
        OI.operatorController::a.whenTrue { relayOn = !relayOn }
        println("relayOn: $relayOn")
    }
}