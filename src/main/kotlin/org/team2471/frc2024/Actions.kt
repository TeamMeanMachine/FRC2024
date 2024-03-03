package org.team2471.frc2024

import kotlinx.coroutines.DelicateCoroutinesApi
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.drive
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
        println("climbing with ${OI.operatorController.rightTrigger} setpoint ${Climb.climbSetpoint} tried to set to ${(OI.operatorRightTrigger * (Climb.MAX_CLIMB_INCHES - Climb.MIN_CLIMB_INCHES) + Climb.MIN_CLIMB_INCHES).inches}")
    }
}

suspend fun spit() = use(Intake) {
    println("starting spit periodic")
    Intake.intakeState = Intake.IntakeState.SPITTING
    Pivot.autoAim = false
    Pivot.angleSetpoint = 45.0.degrees
    periodic {
        if (OI.driverController.leftTriggerFullPress) {
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
//    if (Robot.isAutonomous) {
//        Shooter.rpmTop = Shooter.shootingRpmTop
//        Shooter.rpmBottom = Shooter.shootingRpmBottom
//    }
    val t = Timer()
    if (Robot.isAutonomous) {
        t.start()
        periodic {
            println("combined rpm error: ${(Shooter.motorRpmTop - Shooter.rpmTopSetpoint).absoluteValue + (Shooter.motorRpmBottom - Shooter.rpmBottomSetpoint).absoluteValue}")
            if ((Shooter.motorRpmTop - Shooter.rpmTopSetpoint).absoluteValue + (Shooter.motorRpmBottom - Shooter.rpmBottomSetpoint).absoluteValue < 500.0) {
                println("at rpm shooting now!!! took ${t.get().round(3)} seconds")
                this.stop()
            }

            if (t.get() > 2.0) {
                println("waited 2.0 seconds shooting at lower power")
                this.stop()
            }
        }
    }
    if (Pivot.angleSetpoint != Pivot.AMPPOSE) {
        Intake.intakeMotorTop.setPercentOutput(0.5)
        Intake.intakeMotorBottom.setPercentOutput(0.5)
    }
    Intake.feederMotor.setPercentOutput(1.0)
    t.start()
    periodic {
        if (t.get() > 0.5) {
            println("exiting shooting")
            this.stop()
        }
    }
    Intake.intakeState = Intake.IntakeState.EMPTY
//    Shooter.shooting = false
//    if (!Robot.isAutonomous) {
//        Shooter.rpmTop = 0.0
//        Shooter.rpmBottom = 0.0
//    }
}
suspend fun aimAtSpeaker() {
    Drive.aimTarget = true
    Pivot.autoAim = true

    suspendUntil(20) { !OI.driverController.y }

    Drive.aimTarget = false
    Pivot.autoAim = false

    Pivot.angleSetpoint = Pivot.DRIVEPOSE
    Shooter.rpmTopSetpoint = 0.0
    Shooter.rpmBottomSetpoint = 0.0

}

suspend fun driveToClosestNote() = use(Drive) {
    periodic {
        if (NoteDetector.seesNote) {
            NoteDetector.closestNote?.let {
                Drive.drive(
                    it.robotCoords.normalize(),
                    0.0,
                    false
                )
            }
        }
        if (Intake.holdingCargo) {
            this.stop()
        }
    }
}


suspend fun pickUpSeenNote() = use(Drive, name = "pick up note") {
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

            if (NoteDetector.notes.size == 0) {
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
                val turnControl = sign(headingError) * Drive.parameters.kHeadingFeedForward + headingError * Drive.parameters.kpHeading

                val driveSpeed =  OI.driveLeftTrigger //if (headingError > angleMarginOfError) ((notePos.length - minDist) / 5.0).coerceIn(0.0, OI.driveLeftTrigger) else  OI.driveLeftTrigger

                val driveDirection = Vector2( -notePos.y, notePos.x).normalize()
                Drive.drive(driveDirection * driveSpeed, turnControl, false)

                prevHeadingError = headingError
                prevTime = t

                println("using estimation: $useEstimation")
                println("x: ${notePos.x}, y: ${notePos.y}")
                println("turn control: ${turnControl}, heading err: ${headingError}")


            }

            if (OI.driveLeftTrigger < 0.2) {
                stop()
            } else if (Intake.intakeState != Intake.IntakeState.INTAKING) {
                println("stopped because intake is done")
                println("intake state ${Intake.intakeState.name}")
                stop()
            }
        }

    }
}


