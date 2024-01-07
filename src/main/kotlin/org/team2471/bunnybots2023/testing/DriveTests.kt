package org.team2471.bunnybots2023.testing


import org.team2471.bunnybots2023.Drive
import org.team2471.bunnybots2023.OI
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.motion.following.resetOdometry
import org.team2471.frc.lib.motion.following.tuneDrivePositionController
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.util.Timer
import kotlin.math.absoluteValue

suspend fun Drive.steeringTests() = use(this) {
    println("Got into steeringTests. Hi")
    for (module in 0..3) {
        println("Got into first for. Hi.")
        for (quadrant in 0..4) {
            Drive.modules[module].angleSetpoint = (quadrant * 90.0).degrees
            delay(0.25)
        }
        delay(0.5)
    }
}

suspend fun Drive.steerFeedbackCoefficientTest() = use(this) {
    for (i in 0..10) {
        for (quadrant in 0..4) {
            Drive.modules[0].angleSetpoint = (quadrant * 90.0).degrees
            delay(0.25)
        }
        delay(0.5)
    }
}
//

suspend fun Drive.driveTests() = use(this) {

    for (i in 0..3) {
        Drive.modules[i].setDrivePower(0.5)
        delay(1.0)
        Drive.modules[i].setDrivePower(0.0)
        delay(0.2)
    }
}

suspend fun Drive.fullTest() = use(this) {
    periodic {
        Drive.drive(OI.driveTranslation, OI.driveRotation)
    }
}

suspend fun Drive.tuneDrivePositionController() = use(this) {
    tuneDrivePositionController(OI.driverController)
}

suspend fun Drive.rampTest() = use(Drive) {
    Drive.resetOdometry()
    val t = Timer()
    t.start()
    periodic {
        drive(
            Vector2(0.0, 1.0),
            0.0,
            false
        )
        if (t.get() > 2.0) {
            drive(
                Vector2(0.0, 0.0),
                0.0,
                false
            )
            stop()
        }
    }
    println("endPos: ${Drive.position.y}")
}

suspend fun Drive.aimFTest() = use(Drive) {
    var turn = 0.0
    var upPressed = false
    var downPressed = false
    periodic {
        if (OI.driverController.dPad == Controller.Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == Controller.Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != Controller.Direction.UP && upPressed) {
            upPressed = false
            turn += 0.01
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            turn -= 0.01
        }
        println("turn $turn")
        drive(
            Vector2(0.0, 0.0),
            turn,
            false
        )
    }
}

//suspend fun Drive.driveCircle() = use(Drive) {
//    println("Driving along circle")
//
//    var prevTime = 0.0
//    var circlePosition = position
//    var prevPosition = position
//
//    val timer = Timer()
//    timer.start()
//    var prevPositionError = Vector2(0.0, 0.0)
//    periodic {
//        val t = timer.get()
//        val dt = t - prevTime
//
//        val currRadius = circlePosition.length
//        val currAngle = circlePosition.angle.radians
//
//        circlePosition = Vector2(goalRadius * goalAngle.sin(), goalRadius * goalAngle.cos())
//        val positionSetPoint = circlePosition
//        println("dt=$dt  x = ${positionSetPoint.x} y = ${positionSetPoint.y}")
//
//        // position error
//        val positionError = positionSetPoint - position
//        //println("time=$t   pathPosition=$pathPosition position=$position positionError=$positionError")
//
//        // position feed forward
//        val pathVelocity = (positionSetPoint - prevPosition) / dt
//        prevPosition = positionSetPoint
//
//        // position d
//        val deltaPositionError = positionError - prevPositionError
//        prevPositionError = positionError
//
//        val translationControlField =
//            pathVelocity * parameters.kPositionFeedForward + positionError * parameters.kpPosition + deltaPositionError * parameters.kdPosition
//
//        var turn = 0.0
//        if (Limelight.hasValidTarget) {
//            turn = aimPDController.update(Limelight.aimError)
//        } else {
//            var error = (position.angle.radians - heading).wrap()
//            if (error.asDegrees.absoluteValue > 90.0) error = (error - 180.0.degrees).wrap()
//            turn = aimPDController.update(error.asDegrees)
//        }
//
//        // send it
//        drive(translationControlField, turn, true)
//
//        prevTime = t
//
////        println("Time=$t Path Position=$pathPosition Position=$position")
////        println("DT$dt Path Velocity = $pathVelocity Velocity = $velocity")
//    }
//
//    // shut it down
//    drive(Vector2(0.0, 0.0), 0.0, true)
//}
