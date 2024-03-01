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

    Intake.intakeMotorTop.setPercentOutput(0.5)
    Intake.intakeMotorBottom.setPercentOutput(0.5)
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


suspend fun pickUpSeenNote() = use(Drive, Intake) {
    val angleMarginOfError = 5.0
    val minDist = 2.5
    var noteEstimatedPosition : Vector2 = Vector2(0.0, 0.0)
    var notePosCount : Int = 0
    val notePosMaxError = 2.5
    if (NoteDetector.seesNote) {
        var prevHeadingError = 0.0
        val timer = Timer()
        timer.start()
        var prevTime = 0.0
        periodic {
            val t = timer.get()
            val dt = t - prevTime

            var headingError = 0.0
            var notePos = Vector2(0.0, 0.0)

            var useEstimation = false

            if (!NoteDetector.seesNote) {
                useEstimation = true
            }
            if (!useEstimation) {
                val note = NoteDetector.notes[0]
                notePos = note.robotCoords
                headingError = note.yawOffset
                val fieldPosition = notePos.rotateDegrees(-Drive.heading.asDegrees) + Drive.position

                if ((noteEstimatedPosition/notePosCount.toDouble()-fieldPosition).length > notePosMaxError) { // is it a different note
                    useEstimation = true
                } else {
                    noteEstimatedPosition += fieldPosition
                    notePosCount++
                }
            }
            if (useEstimation && notePosCount > 0) {
                val noteFieldPose = noteEstimatedPosition/notePosCount.toDouble()
                headingError = noteFieldPose.angleAsDegrees
                notePos = (noteEstimatedPosition/notePosCount.toDouble() - Drive.position).rotateDegrees(Drive.heading.asDegrees)
            }

            if (notePos != Vector2(0.0, 0.0)) {

                val headingVelocity = (headingError - prevHeadingError) / dt
                val turnControl = -headingError.sign * Drive.parameters.kHeadingFeedForward + headingError * Drive.parameters.kpHeading + headingVelocity * Drive.parameters.kdHeading

                val driveSpeed = if (headingError > angleMarginOfError) ((notePos.length - minDist) / 5.0).coerceIn(0.0, 1.0) else 1.0

                Drive.drive(notePos.normalize() * driveSpeed, turnControl, false)

                if (notePos.length < minDist) {
                    Intake.intakeMotorTop.setPercentOutput(0.5)
                    Intake.intakeMotorBottom.setPercentOutput(0.5)
                }

                prevHeadingError = headingError
                prevTime = t

            }

            if (OI.driveTranslation.x != 0.0 || OI.driveTranslation.y != 0.0 || OI.driveRotation != 0.0) {
                stop()
            }


        }

    }
}


