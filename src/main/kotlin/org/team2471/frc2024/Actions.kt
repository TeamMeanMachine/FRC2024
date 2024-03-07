package org.team2471.frc2024

import edu.wpi.first.wpilibj.DriverStation
import kotlinx.coroutines.DelicateCoroutinesApi
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.DoubleRange
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.util.Timer
import kotlin.math.absoluteValue
import kotlin.math.sign

suspend fun climbWithTrigger() = use(Climb) {
    println("inside climbWIthTrigger!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    periodic {
        if (OI.operatorRightTrigger < 0.02) {
            this.stop()
        }
        Climb.climbSetpoint = (OI.operatorRightTrigger * (Climb.MAX_CLIMB_INCHES - Climb.MIN_CLIMB_INCHES) + Climb.MIN_CLIMB_INCHES).inches
//        println("climbing with ${OI.operatorController.rightTrigger} setpoint ${Climb.climbSetpoint} tried to set to ${(OI.operatorRightTrigger * (Climb.MAX_CLIMB_INCHES - Climb.MIN_CLIMB_INCHES) + Climb.MIN_CLIMB_INCHES).inches}")
    }
}

suspend fun spit() = use(Intake) {
    println("starting spit periodic")
    Intake.intakeState = Intake.IntakeState.SPITTING
    Pivot.aimSpeaker = false
    Pivot.angleSetpoint = 45.0.degrees
    periodic {
        if (OI.driverController.a) {
            Intake.intakeMotorTop.setPercentOutput(-0.9)
            Intake.intakeMotorBottom.setPercentOutput(-0.9)
            Intake.feederMotor.setPercentOutput(-0.9)
        } else {
            Intake.intakeState = Intake.IntakeState.EMPTY
            this.stop()
        }
    }
}

@OptIn(DelicateCoroutinesApi::class)
suspend fun fire() = use(Shooter, Intake){
    val t = Timer()
//    if (Pivot.angleSetpoint != Pivot.AMPPOSE) {
    Intake.intakeMotorTop.setPercentOutput(1.0)
    Intake.intakeMotorBottom.setPercentOutput(1.0)
//    }
    Intake.feederMotor.setPercentOutput(1.0)
    t.start()
    periodic {
        if (/*(Robot.isAutonomous && !Intake.topBreak && t.get() > 0.5) ||*/ (t.get() > 1.0 && Robot.isAutonomous) || (!Robot.isAutonomous && (OI.driverController.rightTrigger < 0.8))) { //undeployed
            println("exiting shooting")
            this.stop()
        }
    }

    println("Shot note..  Distance ${Pivot.distFromSpeaker.round(2)}  Pivot Setpoint: ${Pivot.angleSetpoint.asDegrees.round(1)}  Pivot Encoder:  ${Pivot.pivotEncoderAngle.asDegrees.round(1)}  ShooterTSetpoint: ${Shooter.rpmTopSetpoint.round(1)}  ShooterTRpm:  ${Shooter.motorRpmTop.round(1)}  ShooterBSetpoint:  ${Shooter.rpmBottomSetpoint.round(1)}  ShooterBRpm:  ${Shooter.motorRpmBottom.round(1)}")

    Intake.intakeState = Intake.IntakeState.EMPTY
    if (Robot.isAutonomous) {
        Drive.aimSpeaker = false
        Pivot.aimSpeaker = false
    }
}
suspend fun aimAtSpeaker() {
    Pivot.revving = true //unsure if this variable necessary. Comment out in pivot if an issue
    Drive.aimSpeaker = true
    Pivot.aimSpeaker = true

    val t = Timer()
    t.start()

    delay(0.1)
    Pivot.revving = false

    if (!DriverStation.isAutonomous()) {
        suspendUntil(20) { !OI.driverController.y }

        Drive.aimSpeaker = false
        Pivot.aimSpeaker = false

        Pivot.angleSetpoint = Pivot.DRIVEPOSE
        Shooter.rpmTopSetpoint = 0.0
        Shooter.rpmBottomSetpoint = 0.0
    }
}

suspend fun aimAndShoot(print: Boolean = false) {
    val t = Timer()
    aimAtSpeaker()
    t.start()
    suspendUntil { Pivot.speakerIsReady(debug = print) || t.get() > 1.0 }
    if (t.get() > 1.0) {
        println("Aiming max time")
        Pivot.speakerIsReady(debug = true)
    }
    fire()
}

suspend fun pickUpSeenNote(speed: Double = -1.0, cautious: Boolean = false, timeOut: Boolean = true) = use(Drive, name = "pick up note") {
    var noteEstimatedPosition : Vector2 = Vector2(0.0, 0.0)
    var notePosCount : Int = 0
    val notePosMaxError = 4

    println("picking up note")

    if (NoteDetector.seesNote) {
        println("sees note")
        var prevHeadingError = 0.0
        val timer = Timer()
        timer.start()
        var prevTime = 0.0
        Intake.intakeState = Intake.IntakeState.INTAKING
        periodic {
            val t = timer.get()
            val dt = t - prevTime

            var headingError = 0.0
            var notePos = Vector2(0.0, 0.0)

            var useEstimation = false

            if (NoteDetector.notes.isEmpty() /*|| notePosCount > 10*/) {
                useEstimation = true
            }
            if (!useEstimation) {
                val note = NoteDetector.notes[0]
                notePos = note.robotCoords
                headingError = note.yawOffset
                val fieldPosition = note.fieldCoords

                if ((noteEstimatedPosition/notePosCount.toDouble()-fieldPosition).length > notePosMaxError) { // is it a different note
                    useEstimation = true
                } else {
                    noteEstimatedPosition += fieldPosition
                    notePosCount++
                }
            }
            if (useEstimation && notePosCount > 0) {
                val noteFieldPose : Vector2 = noteEstimatedPosition/notePosCount.toDouble()
                headingError = 0.0 //(noteFieldPose - Drive.combinedPosition).angleAsDegrees + Drive.heading.asDegrees
                notePos = (noteFieldPose - Drive.combinedPosition).rotateDegrees(-Drive.heading.asDegrees)
            }

            if (notePos != Vector2(0.0, 0.0)) {

                val headingVelocity = (headingError - prevHeadingError) / dt
                val velocityComponent = headingVelocity * Drive.parameters.kdHeading * 0.1
                val turnControl = sign(headingError) * Drive.parameters.kHeadingFeedForward + headingError * Drive.parameters.kpHeading * 1 //+ velocityComponent

                var driveSpeed = if (speed < 0.0 ) OI.driveLeftTrigger else speed //if (headingError > angleMarginOfError) ((notePos.length - minDist) / 5.0).coerceIn(0.0, OI.driveLeftTrigger) else  OI.driveLeftTrigger

                if (cautious) {
                    driveSpeed *= linearMap(0.0, 1.0, 0.2, 1.0, (notePos.length - 2.5) / 5.0).coerceIn(0.0, 1.0)
                }

                val driveDirection = Vector2( -1.5 * notePos.y, notePos.x.coerceIn(0.0, 1.0)).normalize()
                Drive.drive(driveDirection * driveSpeed, turnControl, false)


//                println("using estimation: $useEstimation")
//                println("NOTE x: ${notePos.x}, y: ${notePos.y}")
//                println("Drive Speed $driveSpeed")
//                println("turn control: ${turnControl}, heading err: ${headingError}")
//                println("velocity ${headingVelocity} dt: $dt")
//                println("difference ${headingError - prevHeadingError}")
//                println("vcomponent: ${velocityComponent}")

                prevHeadingError = headingError
                prevTime = t

            }

//            println("combinedx: ${Drive.combinedPosition.x}  notex: ${notePos.x}")
            if (OI.driveLeftTrigger < 0.2 && !Robot.isAutonomous) {
                println("Stopping because ")
                stop()
            } else if (Intake.intakeState != Intake.IntakeState.INTAKING) {
                println("stopped because intake is done, state: ${Intake.intakeState.name}")
                stop()
            } else if (Robot.isAutonomous && timer.get() > 2.0 && timeOut) {
                println("exiting pick up note, its been too long")
                stop()
            }
        }
        Drive.drive(Vector2(0.0, 0.0), 0.0, false)
    }
}

suspend fun lockToAmp() {
//    Drive.aimAmp = true
    println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaAIMAMP ${Drive.aimAmp}")
//    suspendUntil(20) { !OI.driverController.b }
//    Drive.aimAmp = false

    val newPath = Path2D("newPath")
    newPath.addVector2(Drive.combinedPosition)
    if (isBlueAlliance) {
        newPath.addPoint(7.0, 25.0)  // coords??
//        newPath.addPointAndTangent(7.0, 25.0, 0.0, -4.0)
    } else {
        newPath.addPoint(47.0, 25.0)  // coords??
//        newPath.addPointAndTangent(47.0, 25.0, 0.0, -4.0)
    }
    val distance = newPath.length
    val rate = Drive.velocity.length
    val time = distance / rate * 2.0
    newPath.easeCurve.setMarkBeginOrEndKeysToZeroSlope(false)
    newPath.addEasePoint(0.0, 0.0)
    newPath.easeCurve.setMarkBeginOrEndKeysToZeroSlope(true)
    newPath.addEasePoint(time, 1.0)
    newPath.addHeadingPoint(0.0, Drive.heading.asDegrees)
    newPath.addHeadingPoint(time, 90.0)
    Drive.driveAlongPath(newPath) { OI.driverController.b }
}


