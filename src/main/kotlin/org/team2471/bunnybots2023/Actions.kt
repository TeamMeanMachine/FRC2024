package org.team2471.bunnybots2023

import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.util.Timer
import kotlin.math.absoluteValue


suspend fun fire() = use(Shooter) {
    println("FIRING!!!! IM SHOOTING BALL")
    Shooter.setLastShotTime()
    if (Limelight.enemyBuckets.isNotEmpty()) {
        println(
            "limelight specs! Turret Offset: ${Turret.turretError}, ticks ahead: ${Limelight.enemyBuckets[0].ticksToTarget}, bucket velocity: ${Limelight.enemyBuckets[0].vBotCentCoords.length}, botcentcoords: ${Limelight.enemyBuckets[0].botCentCoords}, pBotcentcoords${
                Limelight.enemyBuckets[0].pBotCentCoords(
                    Limelight.enemyBuckets[0].ticksToTarget
                )
            }"
        )
    }
    Shooter.shooterMotor.setPercentOutput(1.0)
    val t = Timer()
    var waitingTime = 0.0
    var previousCurrent = 999.0
    val ballWasLoaded = Intake.detectedBall
//    periodic {
//        val current = Shooter.shooterMotor.current
//        println(listOf(current, previousCurrent))
//        if ((Shooter.shooterMotor.current.absoluteValue - previousCurrent.absoluteValue).absoluteValue < 1.0) {
//            println("current difference less than 1")
//        } else {
//            waitingTime = t.get()
//        }
//        if (t.get() - waitingTime > 0.04) {
//            println("difference less than 1 for 0.05 seconds")
//            this.stop()
//        }
//        previousCurrent = Shooter.shooterMotor.current
//        Shooter.timeAfterLastShot = 0.0
//
//    }
    Shooter.uptakeMotor.setPercentOutput(1.0)
    delay(0.1)
    Shooter.reverseBall = false
    Intake.ballPast = true

}
suspend fun holdFire() = use(Shooter) {
    Shooter.shooterMotor.setPercentOutput(1.0)
    Shooter.uptakeMotor.setPercentOutput(1.0)
    val ballWasLoaded = Intake.detectedBall
    periodic {
        if (!OI.operatorController.rightTriggerFullPress) {
            this.stop()
        }
    }
    Shooter.timeAfterLastShot = 0.0
    Shooter.reverseBall = false
    Intake.ballPast = ballWasLoaded
}
suspend fun toggleBallCollection() = use(Shooter, Intake) {
//    if (Shooter.disableUptake) {
//        Shooter.disableUptake = false
//        Intake.disableConveyor = false
//    } else {
//        Shooter.disableUptake = true
//        Intake.disableConveyor = true
//    }

    if (Shooter.disableUptake) {
        Shooter.disableUptake = false
        Intake.disableConveyor = false
    } else {
        Shooter.disableUptake = true
        Intake.disableConveyor = true
    }
}
suspend fun holdToSpit() = use(Intake, Shooter) {
    val prevIntaking = Intake.intaking
    println("starting periodic")
    periodic {
        if (!OI.driverController.leftBumper) {
            this.stop()
        }
        Intake.centerMotor.setPercentOutput(-0.4)
        Intake.frontMotor.setPercentOutput(-0.5)
        Intake.conveyorMotor.setPercentOutput(-1.0)
        Shooter.uptakeMotor.setPercentOutput(-1.0)
    }
    println("stopping periodic")
    if (prevIntaking) {
        Intake.startIntake()
    } else {
        Intake.stopIntake()
    }
}

suspend fun holdToIntake() = use(Intake) {
    periodic {
        if (!OI.driverController.rightBumper) {
            this.stop()
        }
        Intake.startIntake()
    }
    Intake.stopIntake()
}
