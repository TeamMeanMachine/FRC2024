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
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.motion.following.lookupPose
import org.team2471.frc.lib.motion.following.poseDiff
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
    suspendUntil { Pivot.speakerIsReady(debug = print) || t.get() > 0.75 }
    if (t.get() > 0.75) {
        println("Aiming max time")
        Pivot.speakerIsReady(debug = true)
    }
    fire()
}

suspend fun pickUpSeenNote(speed: Double = -1.0, cautious: Boolean = false, timeOut: Boolean = true, expectedPos : Vector2? = null) = use(Drive, name = "pick up note") {
    try {
        println("picking up note")

        var noteEstimatedPosition: Vector2 = Vector2(0.0, 0.0)
        var notePosCount: Double = 0.0

        var success = false

        val startTime = Timer.getFPGATimestamp()

        var prevHeadingError = 0.0

        // Start intaking
        Intake.intakeState = Intake.IntakeState.INTAKING

        periodic {

            var headingError : Double? = null
            var notePos : Vector2? = null
            var fieldPos : Vector2? = null

            val elapsedTime = Timer.getFPGATimestamp() - startTime

            val estimatedFieldPos = noteEstimatedPosition / notePosCount

            val expectedPosWeight = (10.0/(notePosCount - 7.4) - 0.4).coerceIn(0.0, 1.0)
            val expectedFieldPos = if (expectedPos != null) { expectedPos * expectedPosWeight + noteEstimatedPosition * (1.0 - expectedPosWeight) } else estimatedFieldPos
            val notePosMaxError = 2.5 + (2/notePosCount).coerceIn(0.0, 3.0) // If we don't have many measurements then some more error is acceptable

            var noteFound = false

            for (note in NoteDetector.notes) {
                val latency = Timer.getFPGATimestamp() - note.timestampSeconds
                val previousPose = Drive.lookupPose(note.timestampSeconds)!!
                val poseDiff = Drive.poseDiff(latency)!! // Exclamation marks are probably fine
                val timeAdjustedRobotPose = (note.robotCoords.rotateDegrees(-previousPose.heading.asDegrees) + poseDiff.position).rotateDegrees(-Drive.heading.asDegrees)

                val fieldPosition = note.fieldCoords + poseDiff.position

                if ((expectedFieldPos - fieldPosition).length < notePosMaxError) { // is it a different note
                    notePos = timeAdjustedRobotPose
                    headingError = note.yawOffset + poseDiff.heading.asDegrees
                    fieldPos = fieldPosition
                    noteFound = true
                    break
                }
            }

            if (noteFound) {
                val weight = 1.0
                noteEstimatedPosition += fieldPos!! * weight
                notePosCount += weight

            }

            if (!noteFound && notePosCount == 0.0) {
                println("pick up seen note did not see any note to pickup")
                stop() // we did not see any note
            }

            if (!noteFound && notePosCount != 0.0) {
                headingError = 0.0 //(estimatedFieldPos - Drive.combinedPosition).angleAsDegrees + Drive.heading.asDegrees // <-- This does not work yet, so 0.0
                notePos = (estimatedFieldPos - Drive.combinedPosition).rotateDegrees(-Drive.heading.asDegrees)
            }

            if (notePos != null && headingError != null) {

                val headingVelocity = (headingError - prevHeadingError) / 0.02

                val feedForward = sign(headingError) * 0.001
                val p = headingError * 0.05
                val d = headingVelocity * 0.005

                var driveSpeed = if (speed < 0.0) OI.driveLeftTrigger else speed
                val turnSpeed = feedForward + p + d

                if (cautious) {
                    driveSpeed *= linearMap(0.0, 1.0, 0.2, 1.0, (notePos.length - 2.5) / 5.0).coerceIn(0.0, 1.0)
                }

                val driveDirection = Vector2(-1.5 * notePos.y, notePos.x).normalize()
                Drive.drive(driveDirection * driveSpeed, turnSpeed, false)

                println("using estimation: $noteFound")
                println("NOTE x: ${notePos.x}, y: ${notePos.y}")
                println("Drive Speed $driveSpeed")
                println("turn control: ${turnSpeed}, heading err: ${headingError}")
                println("heading velocity ${headingVelocity}")
                println("difference ${headingError - prevHeadingError}")
                println("pcomponent: $p \nvcomponent: ${d}")
            }

            if (OI.driveLeftTrigger < 0.2 && !Robot.isAutonomous) {
                stop()
            } else if (Intake.intakeState != Intake.IntakeState.INTAKING) {
                println("stopped because intake is done, state: ${Intake.intakeState.name}")
                success = true
                stop()
            } else if (Robot.isAutonomous && timeOut && ((elapsedTime > 2.5 && PoseEstimator.apriltagsEnabled) || elapsedTime > 4.5)) {
                println("exiting pick up note, its been too long")
                stop()
            }



        }
        Drive.drive(Vector2(0.0, 0.0), 0.0, false)
        return@use success

    } catch (exception: Exception) {
        println("error in pickUpSeenNote: $exception")
    }
}

suspend fun lockToAmp() /*= use(Drive)*/ {
    Drive.aimAmp = true
    println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaAIMAMP ${Drive.aimAmp}")
    suspendUntil(20) { !OI.driverController.b }
    Drive.aimAmp = false

    val newPath = Path2D("newPath")
    newPath.addVector2(Drive.combinedPosition)
    if (isBlueAlliance) {
        newPath.addPoint(6.0, 25.0)  // coords??
//        newPath.addPointAndTangent(7.0, 25.0, 0.0, -4.0)
    } else {
        newPath.addPoint(48.0, 25.0)  // coords??
//        newPath.addPointAndTangent(47.0, 25.0, 0.0, -4.0)
    }
    val distance = newPath.length
    val rate = max(Drive.velocity.length, 15.0) // if we are stopped, use 5 fps
    var time = distance / rate * 2.0
    if (time < 0.5) {
        time = 0.5
    }
    newPath.duration = time
    newPath.easeCurve.setMarkBeginOrEndKeysToZeroSlope(false)  // if this doesn't work, we could add with tangent manually
    newPath.addEasePoint(0.0, 0.0)
    newPath.easeCurve.setMarkBeginOrEndKeysToZeroSlope(true)
    newPath.addEasePoint(time, 1.0)
    newPath.addHeadingPoint(0.0, Drive.heading.asDegrees)
    newPath.addHeadingPoint(time * 0.5, 90.0)
    newPath.addHeadingPoint(time, 90.0)
//    Drive.driveAlongPath(newPath, headingOverride = {90.0.degrees}) { !OI.driverController.b }
    println("im at amp")
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
        println("firing note at time: ${timer.get()}  angle: ${Pivot.pivotEncoderAngle}")
        fire(1.0)
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


