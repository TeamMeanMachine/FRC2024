package org.team2471.frc2024

import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.wpilibj.DriverStation
import kotlinx.coroutines.DelicateCoroutinesApi
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.motion_profiling.MotionCurve
import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.units.*
import org.team2471.frc2024.Drive.heading
import org.team2471.frc2024.Drive.isBlueAlliance
import org.team2471.frc2024.Drive.isRedAlliance
import org.team2471.frc2024.Drive.position
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

suspend fun spit() {
    println("starting spit periodic")
    Pivot.aimSpeaker = false
    Pivot.angleSetpoint = 45.0.degrees
    suspendUntil {Pivot.pivotError.absoluteValue < 10.0}
    periodic {
        if (OI.driverController.a) {
            Intake.intakeState = Intake.IntakeState.SPITTING
        } else {
            Intake.intakeState = Intake.IntakeState.EMPTY
            this.stop()
        }
    }
}

@OptIn(DelicateCoroutinesApi::class)
suspend fun fire(duration: Double? = null) = use(Shooter, name = "fire"){
    println("inside fire")
    val t = Timer()
//    if (Pivot.angleSetpoint != Pivot.AMPPOSE) {
    Intake.intakeState = Intake.IntakeState.SHOOTING
//    }
    t.start()
    periodic {
        println("in shoot periotoic")
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
    Drive.aimTarget = AimTarget.SPEAKER
    Pivot.aimSpeaker = true

    if (!DriverStation.isAutonomous()) {
        suspendUntil(20) { !OI.driverController.y }

        Drive.aimTarget = AimTarget.NONE
        Pivot.aimSpeaker = false

        Pivot.angleSetpoint = Pivot.DRIVEPOSE
        Shooter.rpmTopSetpoint = 0.0
        Shooter.rpmBottomSetpoint = 0.0
    }
}

suspend fun aimAtTagDemo() {
    Drive.aimTarget = AimTarget.DEMOTAG
    Pivot.demoAim = true

    suspendUntil(20) { !OI.driverController.y }

    Drive.aimTarget = AimTarget.NONE
    Pivot.demoAim = false

    Pivot.angleSetpoint = Pivot.DRIVEPOSE
    Shooter.rpmTopSetpoint = 0.0
    Shooter.rpmBottomSetpoint = 0.0

}

suspend fun aimFromPodium() {
    Drive.aimTarget = AimTarget.PODIUM
    Pivot.aimSpeaker = true

    suspendUntil { !OI.driverController.start }

    Drive.aimTarget = AimTarget.NONE
    Pivot.aimSpeaker = false

    Pivot.angleSetpoint = Pivot.DRIVEPOSE
    Shooter.rpmTopSetpoint = 0.0
    Shooter.rpmBottomSetpoint = 0.0

}


// I found podium almost works so imma just copy
suspend fun aimForPass() = use(Pivot, name = "Pass") {
    Drive.aimTarget = AimTarget.PASS

    Pivot.angleSetpoint = 39.0.degrees

    Shooter.manualShootState = true

    suspendUntil { !OI.driverController.x }

    Drive.aimTarget = AimTarget.NONE

    Pivot.angleSetpoint = Pivot.DRIVEPOSE

    Pivot.angleSetpoint = Pivot.DRIVEPOSE

    Shooter.manualShootState = false
}


suspend fun aimAndShoot(print: Boolean = false, minTime: Double = 0.7, delay: Double = 0.0, totalTimeThreshold: Double = 0.0) {

    println("Aiming...")

    val t = Timer()
    aimAtSpeaker()
    delay(delay)
    t.start()
    Pivot.readyToShootTimer.start()
    suspendUntil { Pivot.speakerIsReady(debug = print) || t.get() > minTime || (totalTimeThreshold != 0.0 && Robot.totalTimeTaken() > totalTimeThreshold) }
    if (t.get() > minTime) {
        println("Aiming max time. ${t.get()}")
        Pivot.speakerIsReady(debug = true)
    }
    if (totalTimeThreshold != 0.0 && Robot.totalTimeTaken() > totalTimeThreshold) {
        println("shooting to satisfy shot at $totalTimeThreshold totalTime")
//        Pivot.speakerIsReady(debug = true)
    }
    println("firing note at ${t.get()} seconds and ${Robot.totalTimeTaken()} total time")
    fire()
    Drive.aimTarget = AimTarget.NONE
}

suspend fun seeAndPickUpSeenNote(timeOut: Boolean = true, cancelWithTrigger : Boolean = false) {

    val t = Timer()
    t.start()

    var seesNote = false
    var noteSeenThreshold = 0

    periodic{
        if (NoteDetector.notes.size > 0){
            noteSeenThreshold ++
            if (noteSeenThreshold > 10) {
                seesNote = true
                println("exiting with seen note")
                stop()
            }
            println("i see note for $noteSeenThreshold ticks")
        } else {
            noteSeenThreshold = 0
        }
        if (timeOut && t.get() > 3) {
            println("exiting with timeout")
            stop()
        }
        if (cancelWithTrigger && OI.driveLeftTrigger < 0.2) {
            println("exiting with trigger")
            stop()
        }
    }

    if (seesNote) {
        pickUpSeenNote(cautious = true)
    }
}

suspend fun pickUpSeenNote(cautious: Boolean = true, expectedPos: Vector2? = null, doTurn: Boolean = true, wantedApproachAngle: Double? = null, stopWhenBeamBreak: Boolean = false, ignoreWrongSide: Boolean = false, overrideTimeout: Double? = null, getClosestNoteAtPosition: Vector2? = null) = use(Drive, name = "pick up note") {
//    try {
    println("inside \"pickUpSeenNote\"")

    var noteEstimatedPosition: Vector2 = Vector2(0.0, 0.0)

    val newMeasurementWeight : Double = 0.07

    var noteFoundFlag = false
    var intakeTurnedOn = false

    var success = false

    val startTime = Timer.getFPGATimestamp()

    var prevHeadingError = 0.0
    var noNoteCounter = 0
    var tooFarCounter = 0
    var beamBreakCounter = 0


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
        var newNoteList = NoteDetector.notes

        if (newNoteList.isEmpty()) {
            println("there are no notes inside the note list")
        } else {
            if (getClosestNoteAtPosition != null) {
                val n = NoteDetector.getNearNoteAtPosition(getClosestNoteAtPosition)

                if (n != null) {
                    newNoteList = listOf(n)
                } else {
                    println("i see note but not in expected position")
                    newNoteList = listOf()
                }
            }
        }


        //checking if note is in expected range and setting its position
        for (note in newNoteList) {
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
            println("notes: $newNoteList")

            noNoteCounter += 1

            if (noNoteCounter > 10) {
                println("No notes > 10")
                stop() // we did not see any note
            }
        } else {

            //start driving

            noteFoundFlag = true

            if (!noteFound) {
                headingError = 0.0 //(estimatedFieldPos - Drive.combinedPosition).angleAsDegrees + Drive.heading.asDegrees // <-- This does not work yet, so 0.0
                notePos = (estimatedFieldPos - AprilTag.position.asFeet).rotateDegrees(-Drive.heading.asDegrees)
                fieldPos = estimatedFieldPos
            }


            var approachAngle : Double? = null
            if (wantedApproachAngle != null) {
                approachAngle = wantedApproachAngle
            } else {
//                approachAngle = Drive.heading.asDegrees.coerceIn()
            }

            val targetHeadingError = if (approachAngle != null) { (Drive.heading - approachAngle.degrees).wrap().asDegrees } else headingError
//             println("approachangle = $approachAngle")

            if (notePos != null && targetHeadingError != null && fieldPos != null) {

                val headingVelocity = (targetHeadingError - prevHeadingError) / 0.02

                val feedForward = sign(targetHeadingError) * 0.01
                val p = targetHeadingError * 0.005
                val d = headingVelocity * 0.0005

                var driveSpeed = 1.0
                val turnSpeed = if (doTurn) feedForward + p else 0.0 //+ d

                var targetPose = if (cautious) (notePos - notePos.normalize() * 12.5.inches.asFeet) else (notePos - NoteDetector.camRobotCoords)

                if (approachAngle != null) {
                    targetPose = targetPose.rotateDegrees(approachAngle + 90.0)
                }

                val driveVelocity = Drive.velocity.length

                if (cautious) {
                    driveSpeed *= linearMap(0.0, 1.0, 0.7, 1.0, ((targetPose.length - 1.5) / 5.5).coerceIn(0.0, 1.0))
                    val driveD = (driveVelocity / targetPose.length * 0.1).coerceIn(0.0, 1.0)//linearMap(0.0, 4.5, 0.0, 1.0, 1.0 - ((notePos.length/5.0).coerceIn(0.0, 1.0))))
                    driveSpeed -= driveD
                }
                if (targetPose.x < 3.0 && !intakeTurnedOn) {
                    Intake.intakeState = Intake.IntakeState.INTAKING
                    intakeTurnedOn = true
                }

                if (!Robot.isAutonomous) {
                    driveSpeed *= OI.driveLeftTrigger
                }
                driveSpeed.coerceIn(0.0, 1.0)



                if (cautious) {
                    val driveDirection = Vector2(-0.85 * targetPose.y, targetPose.x).normalize()

                    Drive.drive(driveDirection * driveSpeed, if (turnSpeed == 0.0) OI.driveRotation else turnSpeed, false, closedLoopHeading = !doTurn)
                } else {
                    val driveDirection = Vector2(-targetPose.y, targetPose.x).normalize().rotateDegrees(-Drive.heading.asDegrees)

                    Drive.drive(driveDirection, turnSpeed, false, false)
                }

//                println("translation. direction: $driveDirection  speed $driveSpeed")

//                    println("note Found: $noteFound")
//                    println("NOTE x: ${notePos.x}, y: ${notePos.y}")
//                    println("FIELD x: ${fieldPos.x}, y: ${fieldPos.y}")
//                    println("estimated pos x: ${estimatedFieldPos.x} y: ${estimatedFieldPos.y}")
//                    println("Note pos length: ${notePos.length} Pos velocity: ${driveVelocity}")
//                    println("Drive d: ${driveD}")
//                println("Drive Speed $driveSpeed")
//                println("turn control: ${turnSpeed}, heading err: ${targetHeadingError}")
//                println("heading velocity ${headingVelocity}")
//                println("difference ${targetHeadingError - prevHeadingError}")
//                println("pcomponent: $p \nvcomponent: ${d}")

                prevHeadingError = targetHeadingError
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
            } else if (Robot.isAutonomous && (elapsedTime > (overrideTimeout ?: 5.0))) {
                println("exiting pick up note, it's been too long")
                stop()
            } else if (!ignoreWrongSide && Robot.isAutonomous && ((fieldCoords.x > 30.0 && isBlueAlliance) || (fieldCoords.x < 24.0 && isRedAlliance))) {
                tooFarCounter += 1

                if (tooFarCounter > 5) {
                    println("exiting pick up note, it's on the wrong side  x: ${fieldCoords.x}")
                    stop()
                }
            } else if (stopWhenBeamBreak && Intake.bottomBreak) {
                beamBreakCounter ++
                if (beamBreakCounter > 1) {
                    println("exiting pick up note, beam break activated")
                    stop()
                }
            } else {
                beamBreakCounter = 0
                tooFarCounter = 0
            }
        }
    }

    Drive.drive(Vector2(0.0, 0.0), 0.0, false)
//    if (Robot.isAutonomous) {
//        Drive.xPose()
//        delay(0.05)
//    }
    return@use success

//    } catch (exception: Exception) {
//        println("error in pickUpSeenNote: \n$exception")
//    }
}

suspend fun lockToAmp() = use(Drive) {
    val ampPos = Vector2L(13.85.meters, 7.87.meters)
//    val posDelta = AprilTag.position - ampPos
//    val rotDelta = heading - 90.0.degrees
    println("inside lockToAmp()")
    Drive.aimTarget = AimTarget.AMP
    Pivot.angleSetpoint = Pivot.AMPPOSE
    println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaAIMAMP ${Drive.aimTarget}")
    Drive.aimTarget = AimTarget.NONE

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

suspend fun holdRampUpShooter() {
    val t = Timer()
    t.start()
    periodic {
        Shooter.manualShootState = (Intake.intakeState != Intake.IntakeState.SLOWING && Intake.intakeState != Intake.IntakeState.INTAKING) //true if not intaking a note
        if (!OI.operatorController.leftTriggerFullPress && t.get() > 0.2) {
            this.stop()
        }
    }
    Shooter.manualShootState = false
}

suspend fun toggleAimAtNote() {
    Drive.aimTarget = AimTarget.GAMEPIECE
    val t = Timer()
    suspendUntil { OI.driverController.leftTrigger < 0.2 && t.get() > 0.2 }
    Drive.aimTarget = AimTarget.NONE
}

suspend fun driveAlongChoreoPath(
    path: ChoreoTrajectory,
    resetOdometry: Boolean = false,
    useAprilTag: Boolean = false,
    extraTime: Double = 0.0,
    inResetGyro: Boolean? = null,
    turnOverride: () -> Double? = {null},
    earlyExit: (percentComplete: Double) -> Boolean = {false}
) = use(Drive, name = "DriveAlongChoreoPath") {
    println("inside driveAlongChoreoPath")

    if (inResetGyro ?: resetOdometry) {
        println("Heading = ${Drive.heading}")
        Drive.resetHeading()
        Drive.heading = path.initialPose.rotation.asAngle// * if (flipped) -1.0 else 1.0 //path.headingCurve.getValue(0.0).degrees
        println("After Reset Heading = ${Drive.heading}")
    }

    if (resetOdometry) {
        println("Position = $position")
//        Drive.odometryReset()
        println("Position after odometryReset = $position")

        // set to the numbers required for the start of the path
        position = path.initialPose.translation.asVector2().meters.asFeet
//        AprilTag.position = path.initialPose.translation.asVector2().meters
//        prevPosition = position

        println("After Reset Position = $position")
    }


    var prevTime = -0.2

    val timer = org.team2471.frc.lib.util.Timer()
    timer.start()
    var prevPathPosition = path.initialPose.translation.asVector2().meters
    var prevPathHeading = path.initialPose.rotation.asAngle
    var prevPositionError = Vector2(0.0, 0.0).meters
    var prevHeadingError = 0.0.degrees
    suspendUntil(10) { timer.get() != 0.0}
    println("entering drive periodic")
    periodic {
        val t = timer.get()
        val dt = if (t - prevTime != 0.0) t - prevTime else 0.02
        val pathSample = path.sample(t)

        // position error
        val pathPosition = Vector2(pathSample.x, pathSample.y).meters//path.getPosition(t)
        val currentPosition = if (useAprilTag) AprilTag.position else position.feet
        val positionError = pathPosition - currentPosition
//        println("time=$t   dt=$dt    pathPosition=$pathPosition position=$currentPosition positionError=$positionError")

        // position feed forward
        val pathVelocity = Vector2(pathSample.velocityX, pathSample.velocityY).meters//(pathPosition - prevPathPosition) / dt
//        val pathVelocity = (pathPosition - prevPathPosition) / dt
        prevPathPosition = pathPosition

        // position d
        val deltaPositionError = positionError - prevPositionError
        prevPositionError = positionError

        var translationControlField =
            pathVelocity.asFeet * Drive.parameters.kPositionFeedForward + positionError.asFeet * Drive.parameters.kpPosition + deltaPositionError.asFeet * Drive.parameters.kdPosition

        translationControlField = Vector2(-translationControlField.y, translationControlField.x)
//        println("translationControlField = $translationControlField")


        // heading error
        val robotHeading = Drive.heading
        val pathHeading = pathSample.heading.radians//  * if (flipped) -1.0 else 1.0//path.getAbsoluteHeadingDegreesAt(t).degrees
        val headingError = (robotHeading - pathHeading).wrap()
//        println("Heading Error: $headingError. pathHeading: $pathHeading")

        // heading feed forward
//        val headingVelocity = (pathHeading.asDegrees - prevPathHeading.asDegrees) / dt
        val headingVelocity = pathSample.angularVelocity.radians.asDegrees// * if (flipped) -1.0 else 1.0//(pathHeading.asDegrees - prevPathHeading.asDegrees) / dt
        prevPathHeading = pathHeading

        // heading d
        val deltaHeadingError = headingError - prevHeadingError
        prevHeadingError = headingError

        val turnControl = headingVelocity * Drive.parameters.kHeadingFeedForward + headingError.asDegrees * Drive.parameters.kpHeading + deltaHeadingError.asDegrees * Drive.parameters.kdHeading
//        println("Turn Control: $turnControl")
        if (turnControl.isNaN() || translationControlField.y.isNaN() || translationControlField.x.isNaN()) {
            println("turnControl: $turnControl")
            println("translationControlField $translationControlField")
            println("dt: $dt")

//            throw IllegalArgumentException("requestedVolts == NaN")
        }

        // send it
        Drive.drive(translationControlField, turnOverride() ?: turnControl, true)

        // are we done yet?
        if (t >= path.totalTime + extraTime) {
            println("exiting path")
            stop()
        }
        if (earlyExit(t / path.totalTime)) {
            println("early exiting path. time: $t  duration: ${path.totalTime} percent complete: ${t / path.totalTime}")
            stop()
        }
        prevTime = t

//        println("Time=$t Path Position=$pathPosition Position=$position")
//        println("DT$dt Path Velocity = $pathVelocity Velocity = $velocity")
    }
    println("at the end of driveAlongChoreoPath")

    // shut it down
    Drive.drive(Vector2(0.0, 0.0), 0.0, true)
//    actualRoute.setDoubleArray(doubleArrayOf())
//    plannedPath.setString("")
}

