package org.team2471.frc2024

import edu.wpi.first.wpilibj.DriverStation
import kotlinx.coroutines.DelicateCoroutinesApi
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.math.asFeet
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.motion.following.poseDiff
import org.team2471.frc2024.Drive.isBlueAlliance
import org.team2471.frc2024.Drive.isRedAlliance
import kotlin.math.absoluteValue
import kotlin.math.max
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
suspend fun fire(duration: Double? = null) = use(Shooter){
    val t = Timer()
//    if (Pivot.angleSetpoint != Pivot.AMPPOSE) {
    Intake.intakeState = Intake.IntakeState.SHOOTING
//    }
    t.start()
    periodic {
        Intake.intakeState = Intake.IntakeState.SHOOTING
        if ((t.get() > 0.3 && Robot.isAutonomous) && duration == null) {
            println("exiting shooting from autonomous")
            this.stop()
        }
        if (!Robot.isAutonomous && (OI.driverController.rightTrigger < 0.1) && t.get() > 0.1 && duration == null) {
            println("exiting shooting from released trigger")
            this.stop()
        }
        if (duration != null && t.get() > duration) {
            println("exiting shooting from exceeded set duration of $duration")
            this.stop()
        }
    }

    println("Shot note..  Distance ${Pivot.distFromSpeaker.round(2)}  Pivot Setpoint: ${Pivot.angleSetpoint.asDegrees.round(1)}  Pivot Encoder:  ${Pivot.pivotEncoderAngle.asDegrees.round(1)}  ShooterTSetpoint: ${Shooter.rpmTopSetpoint.round(1)}  ShooterTRpm:  ${Shooter.motorRpmTop.round(1)}  ShooterBSetpoint:  ${Shooter.rpmBottomSetpoint.round(1)}  ShooterBRpm:  ${Shooter.motorRpmBottom.round(1)}")

    Intake.intakeState = Intake.IntakeState.EMPTY
//    if (Robot.isAutonomous) {
//        Drive.aimSpeaker = false
//        Pivot.aimSpeaker = false
//    }
}
suspend fun aimAtSpeaker() {
    Drive.aimSpeaker = true
    Pivot.aimSpeaker = true

    if (!DriverStation.isAutonomous()) {
        suspendUntil(20) { !OI.driverController.y && !OI.driverController.x}

        Drive.aimSpeaker = false
        Pivot.aimSpeaker = false

        Pivot.angleSetpoint = Pivot.DRIVEPOSE
        Shooter.rpmTopSetpoint = 0.0
        Shooter.rpmBottomSetpoint = 0.0
    }
}

suspend fun aimAndShoot(print: Boolean = false, minTime: Double = 0.75) {

    println("Aiming...")

    val t = Timer()
    aimAtSpeaker()
    t.start()
    suspendUntil { Pivot.speakerIsReady(debug = print) || t.get() > minTime }
    if (t.get() > 0.75) {
        println("Aiming max time")
        Pivot.speakerIsReady(debug = true)
    }
    fire()
    Drive.aimSpeaker = false
}

suspend fun seeAndPickUpSeenNote(timeOut: Boolean = true, cancelWithTrigger : Boolean = false) {

    val startTime = Timer.getFPGATimestamp()

    var seesNote = false
    periodic{
        if (NoteDetector.notes.size > 0){
            seesNote = true
            stop()
        }
        if (timeOut && (Timer.getFPGATimestamp() - startTime) > 3) {
            stop()
        }
        if (cancelWithTrigger && OI.driveLeftTrigger < 0.2) {
            stop()
        }
    }

    if (seesNote) {
        pickUpSeenNote(cautious = true)
    }
}

suspend fun pickUpSeenNote(cautious: Boolean = true, timeOut: Boolean = true, expectedPos: Vector2? = null, doTurn: Boolean = true) = use(Drive, name = "pick up note") {
//    try {
    println("inside \"pickUpSeenNote\"")

    var noteEstimatedPosition: Vector2 = Vector2(0.0, 0.0)

    val newMeasurementWeight : Double = 0.07

    var noteFoundFlag = false
    var intakeTurnedOn = false

    var success = false

    val startTime = Timer.getFPGATimestamp()

    var prevHeadingError = 0.0

    if (!Robot.isAutonomous) {
        // Start intaking
        Intake.intakeState = Intake.IntakeState.INTAKING
    }


    periodic {

        var headingError : Double? = null
        var notePos : Vector2? = null
        var fieldPos : Vector2? = null

        val elapsedTime = Timer.getFPGATimestamp() - startTime

        val estimatedFieldPos = noteEstimatedPosition

//            val expectedPosWeight = (10.0/(notePosCount - 7.4) - 0.4).coerceIn(0.0, 1.0)
        var expectedFieldPos : Vector2? = expectedPos
        var notePosMaxError = 3.5
        if (noteFoundFlag) {
//            expectedFieldPos = estimatedFieldPos
//                notePosMaxError = 3.5 // If we don't have many measurements then some more error is acceptable
        }

        var noteFound = false

        if (NoteDetector.notes.isEmpty()) {
            println("there are no notes inside the note list")
        }

        //checking if note is in expected range and setting its position
        for (note in  NoteDetector.notes) {
            val latency = Timer.getFPGATimestamp() - note.timestampSeconds
            val previousPose = Drive.lookupPose(note.timestampSeconds)
            val poseDiff = Drive.poseDiff(latency)

            var tempNotePose: Vector2
            var tempFieldPosition: Vector2
            var tempHeadingErr: Double

            if (poseDiff != null && previousPose != null) {
                tempNotePose = (note.robotCoords.rotateDegrees(previousPose.heading.asDegrees) - poseDiff.position).rotateDegrees(-Drive.heading.asDegrees)
                tempFieldPosition = note.fieldCoords
                tempHeadingErr = note.yawOffset + poseDiff.heading.asDegrees
//                    println("latency: $latency")
//                    println("previous pose: $previousPose")
//                    println("poseDiff: $poseDiff")
//                    println("original note x: ")
            } else {
                tempNotePose = note.robotCoords
                tempFieldPosition = note.fieldCoords
                tempHeadingErr = note.yawOffset
//                    println("NOT USING LATENCY ADJUSTMENT")
            }

//                println("note field pos x: ${tempFieldPosition.x} y: ${tempFieldPosition.y}")

            if (!noteFoundFlag) {
                println("Note found")
                println("Field coord: x: ${tempFieldPosition.x} y: ${tempFieldPosition.y}")
                println("Robot Coords: x ${tempNotePose.x} y: ${tempNotePose.y}")
                if (expectedFieldPos != null) {
                    println("Distance from expected: ${(expectedFieldPos - tempFieldPosition).length}")
                }
            }


            if ((expectedFieldPos != null && (expectedFieldPos - tempFieldPosition).length < notePosMaxError) || expectedFieldPos == null) { // is it a different note
//                    println("note passed check")
                notePos = tempNotePose
                headingError = tempHeadingErr
                fieldPos = tempFieldPosition
                noteFound = true
                break
            } else {
                println("note too far from expected position. notePos: $notePos  expectedFieldPos $expectedFieldPos")
            }
        }

        // Update estimated Position
        if (noteFound) {
            if (!noteFoundFlag) {
                noteEstimatedPosition = fieldPos!!
            } else {
                noteEstimatedPosition *= (1 - newMeasurementWeight)
                noteEstimatedPosition += (fieldPos!! * newMeasurementWeight)
            }

//                val weight = 1.0
//                noteEstimatedPosition += weight * fieldPos
//                notePosCount += weight

        } else {
            println("did not find note")
        }

        if (!noteFound && !noteFoundFlag) {
            println("pick up seen note did not see any note to pickup")
            println("notes: ${NoteDetector.notes}")

            stop() // we did not see any note
        } else {

            //start driving

            noteFoundFlag = true

            if (!noteFound) {
                headingError = 0.0 //(estimatedFieldPos - Drive.combinedPosition).angleAsDegrees + Drive.heading.asDegrees // <-- This does not work yet, so 0.0
                notePos = (estimatedFieldPos - Drive.combinedPosition.asFeet).rotateDegrees(-Drive.heading.asDegrees)
                fieldPos = estimatedFieldPos
            }

            if (notePos != null && headingError != null && fieldPos != null) { // This should always be true but whatever

                val headingVelocity = (headingError - prevHeadingError) / 0.02

                val feedForward = sign(headingError) * 0.001
                val p = headingError * 0.005
                val d = headingVelocity * 0.005

                var driveSpeed = if (!Robot.isAutonomous) OI.driveLeftTrigger else 1.0
                val turnSpeed = if (doTurn) feedForward + p else 0.0 //+ d

                val driveVelocity = Drive.velocity.length
                val driveD = (driveVelocity / notePos.length * 0.1).coerceIn(0.0, 1.0)//linearMap(0.0, 4.5, 0.0, 1.0, 1.0 - ((notePos.length/5.0).coerceIn(0.0, 1.0))))

                if (cautious) {
                    driveSpeed *= linearMap(0.0, 1.0, 0.7, 1.0, ((notePos.length - 1.5) / 5.5).coerceIn(0.0, 1.0))
                    driveSpeed -= driveD
                }
                if (notePos.x < 3.0 && !intakeTurnedOn) {
                    Intake.intakeState = Intake.IntakeState.INTAKING
                    intakeTurnedOn = true
                }

                driveSpeed.coerceIn(0.0, 1.0)

                val driveDirection = Vector2(-0.85 * notePos.y, notePos.x).normalize()
                Drive.drive(driveDirection * driveSpeed, turnSpeed, false, closedLoopHeading = !doTurn)

//                    println("note Found: $noteFound")
//                    println("NOTE x: ${notePos.x}, y: ${notePos.y}")
//                    println("FIELD x: ${fieldPos.x}, y: ${fieldPos.y}")
//                    println("estimated pos x: ${estimatedFieldPos.x} y: ${estimatedFieldPos.y}")
//                    println("Note pos length: ${notePos.length} Pos velocity: ${driveVelocity}")
//                    println("Drive d: ${driveD}")
//                println("Drive Speed $driveSpeed")
//                println("turn control: ${turnSpeed}, heading err: ${headingError}")
//                println("heading velocity ${headingVelocity}")
//                println("difference ${headingError - prevHeadingError}")
//                println("pcomponent: $p \nvcomponent: ${d}")

                prevHeadingError = headingError
            }

            var fieldCoords = NoteDetector.robotCoordsToFieldCoords(notePos ?: Vector2(2.0, 0.0))

            if (Intake.holdingCargo) {
                println("stopped because intake is done, state: ${Intake.intakeState.name}")
                println("time to pick up note: $elapsedTime")
                success = true
                stop()
            } else if (OI.driveLeftTrigger < 0.2 && !Robot.isAutonomous) {
                if (Intake.intakeState == Intake.IntakeState.INTAKING) {
                    Intake.intakeState = Intake.IntakeState.EMPTY
                }
                stop()
            } else if (Robot.isAutonomous && ((timeOut && elapsedTime > 5.0)/* || (!noteFound && notePos!!.length < 0.5)*/)) {
                println("exiting pick up note, it's been too long")
                stop()
            } else if (Robot.isAutonomous && ((fieldCoords.x > 30.0 && isBlueAlliance) || (fieldCoords.x < 24.0 && isRedAlliance))) {
                println("exiting pick up note, it's on the wrong side")
                stop()
            }
        }
    }
    Drive.drive(Vector2(0.0, 0.0), 0.0, false)
    return@use success

//    } catch (exception: Exception) {
//        println("error in pickUpSeenNote: \n$exception")
//    }
}

suspend fun lockToAmp() {
    Drive.aimAmp = true
    println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaAIMAMP ${Drive.aimAmp}")
    suspendUntil(20) { !OI.driverController.b }
    Drive.aimAmp = false

/*    val newPath = Path2D("newPath")
    newPath.addVector2(Drive.combinedPosition.asFeet)
    if (isBlueAlliance) {
        newPath.addPoint(7.0, 25.0)  // coords??
//        newPath.addPointAndTangent(7.0, 25.0, 0.0, -4.0)
    } else {
        newPath.addPoint(47.0, 25.0)  // coords??
//        newPath.addPointAndTangent(47.0, 25.0, 0.0, -4.0)
    }
    val distance = newPath.length
    val rate = max(Drive.velocity.length, 5.0) // if we are stopped, use 5 fps
    val time = distance / rate * 2.0
    newPath.duration = time
    newPath.easeCurve.setMarkBeginOrEndKeysToZeroSlope(false)  // if this doesn't work, we could add with tangent manually
    newPath.addEasePoint(0.0, 0.0)
    newPath.easeCurve.setMarkBeginOrEndKeysToZeroSlope(true)
    newPath.addEasePoint(time, 1.0)
    newPath.addHeadingPoint(0.0, Drive.heading.asDegrees)
    newPath.addHeadingPoint(time, 90.0)
    Drive.driveAlongPath(newPath) { OI.driverController.b }*/
}

suspend fun flipAmpShot() = use(Pivot) {
    val endingAngle = Pivot.MAXHARDSTOP
    val startingAngle = Pivot.AMPPOSE
    val pivotAngleRate = Pivot.pivotAmpRate.getDouble(80.0) //deg per second
    val shootingAngleThreshold: Angle = 90.0.degrees

    val pivotCurve = MotionCurve()
    val curveTime = (startingAngle - endingAngle).asDegrees.absoluteValue / pivotAngleRate

    pivotCurve.storeValue(0.0, startingAngle.asDegrees)
    pivotCurve.storeValue(curveTime, endingAngle.asDegrees)


    //start animating
    val timer = Timer()
    timer.start()
    parallel({
        suspendUntil { Pivot.pivotEncoderAngle > shootingAngleThreshold }
        println("firing note at time: ${timer.get()}  angle: ${Pivot.pivotEncoderAngle}")
        fire()
    }, {
        periodic {
            val t = timer.get()

            Pivot.angleSetpoint = pivotCurve.getValue(t).degrees

            if (t > curveTime) {
                this.stop()
            }
        }
    })


    println("finished pivot animation of ${curveTime.round(1)} seconds, took ${timer.get().round(1)} seconds. pivotAngle: ${Pivot.pivotEncoderAngle} endingAngle: $endingAngle")
}


