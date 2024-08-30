package org.team2471.frc2024

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.input.*
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.AprilTag.resetCameras
import org.team2471.frc2024.Drive.isBlueAlliance
import org.team2471.frc2024.Shooter.manualShootState
import kotlin.math.absoluteValue

object OI : Subsystem("OI") {
    val driverController = XboxController(0)
    val operatorController = XboxController(1)

    private val deadBandDriver = 0.06
    private val deadBandOperator = 0.1

    private val driveTranslationX: Double
        get() = driverController.leftThumbstickX.deadband(deadBandDriver).squareWithSign()

    private val driveTranslationY: Double
        get() = -driverController.leftThumbstickY.deadband(deadBandDriver).squareWithSign()

    val driveTranslation: Vector2
        get() = if (isBlueAlliance) Vector2(driveTranslationX, driveTranslationY) else -Vector2(driveTranslationX, driveTranslationY)  //does owen want this cubed?

    val driveRotation: Double
        get() = (driverController.rightThumbstickX.deadband(deadBandDriver)).cube() // * 0.6

    val driveLeftTrigger: Double
        get() = driverController.leftTrigger

    val driveLeftTriggerFullPress: Boolean
        get() = driverController.leftTriggerFullPress

    val driveRightTrigger: Double
        get() = driverController.rightTrigger

    val operatorLeftTrigger: Double
        get() = operatorController.leftTrigger

    val operatorLeftY: Double
        get() = operatorController.leftThumbstickY.deadband(0.2)

    val operatorLeftX: Double
        get() = operatorController.leftThumbstickX.deadband(0.2)

    val operatorRightTrigger: Double
        get() = operatorController.rightTrigger

    val operatorRightX: Double
        get() = operatorController.rightThumbstickX.deadband(0.2)

    val operatorRightY: Double
        get() = operatorController.rightThumbstickY.deadband(0.0)

    val opX: Boolean
        get() = operatorController::x.get()

    init {
        driverController::back.whenTrue {
            Drive.zeroGyro()
            Drive.initializeSteeringMotors() //not needed 02/05
        }

        driverController::leftBumper.whenTrue {
            if (Intake.intakeState == Intake.IntakeState.EMPTY) {
                Intake.intakeState = Intake.IntakeState.INTAKING
//                Shooter.manualShootState = false
            } else {
                Intake.intakeState = Intake.IntakeState.EMPTY
            }
        }
        driverController::a.whenTrue { spit() }
        driverController::rightTriggerFullPress.whenTrue { /*if (Pivot.angleSetpoint > Pivot.AMPPOSE - 10.0.degrees) flipAmpShot() else*/ fire() }
        driverController::rightBumper.whenTrue { Shooter.manualShootState = !Shooter.manualShootState }
        driverController::y.whenTrue {
            aimAtSpeaker()
//            if (Drive.demoMode) aimAtTagDemo() else aimAtSpeaker();
//            Shooter.manualShootState = !Shooter.manualShootState
        }
        driverController::y.whileTrue {
            manualShootState = true
//            println("aargh why")
        }
        driverController::y.whenFalse {
            manualShootState = false
        }
        driverController::x.whenTrue {
            if (Drive.demoMode) {
                Pivot.angleSetpoint = Pivot.demoAngleEntry.getDouble(Pivot.DEMO_POSE.asDegrees).degrees }
            else {
                aimFromPodium() }
        }
//        driverController::x.whenTrue { Drive.xPose(); println("xpose") }

        driverController::leftTriggerFullPress.whenTrue { // broken right now, see NoteDetector.angleToClosestNote
            seeAndPickUpSeenNote(false, true)
//            toggleAimAtNote()
        }
        driverController::b.whenTrue { if (!Drive.demoMode) lockToAmp() else Pivot.angleSetpoint = Pivot.AMPPOSE }// }
        operatorController::back.whenTrue { resetCameras() }
        operatorController::y.whenTrue { Pivot.angleSetpoint = Pivot.AMPPOSE }
        operatorController::b.whenTrue { Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE }
        operatorController::a.whenTrue { Pivot.angleSetpoint = Pivot.DRIVEPOSE }

        operatorController::leftTriggerFullPress.whenTrue { holdRampUpShooter() }
        ({operatorRightTrigger > 0.03}).whenTrue { println("climbinggggggggggggggggggg"); climbWithTrigger() }
        ({operatorController.leftBumper && operatorController.rightBumper}).whenTrue { println("LOCKING NOWWWWWWWWWWWW!!!!"); Climb.activateRelay() }

        ({ driverController.dPad == Controller.Direction.UP}).whenTrue {
            val newAngle = Pivot.demoAngleEntry.getDouble(Pivot.DEMO_POSE.asDegrees) + 2.0
            if (newAngle < 90.0) {
                Pivot.demoAngleEntry.setDouble(newAngle)
                Pivot.angleSetpoint = newAngle.degrees
            }
//            Shooter.topAmpRPMEntry.setDouble(Shooter.topAmpRPMEntry.getDouble(1200.0) + 100.0)
//            Shooter.bottomAmpRPMEntry.setDouble(Shooter.bottomAmpRPMEntry.getDouble(1200.0) + 100.0)
        }
        ({ driverController.dPad == Controller.Direction.DOWN}).whenTrue {
            val newAngle = Pivot.demoAngleEntry.getDouble(Pivot.DEMO_POSE.asDegrees) - 2.0
            if (newAngle < 90.0) {
                Pivot.demoAngleEntry.setDouble(newAngle)
                Pivot.angleSetpoint = newAngle.degrees
            }
//            Shooter.topAmpRPMEntry.setDouble(Shooter.topAmpRPMEntry.getDouble(1200.0) - 100.0)
//            Shooter.bottomAmpRPMEntry.setDouble(Shooter.bottomAmpRPMEntry.getDouble(1200.0) - 100.0)
        }
        ({ driverController.dPad == Controller.Direction.RIGHT}).whenTrue {
            Shooter.demoRPMEntry.setDouble(Shooter.demoRPMEntry.getDouble(2500.0) + 250.0)
        }
        ({ driverController.dPad == Controller.Direction.LEFT}).whenTrue {
            Shooter.demoRPMEntry.setDouble(Shooter.demoRPMEntry.getDouble(2500.0) - 250.0)
        }

//        ({ driverController.dPad == Controller.Direction.UP }).whenTrue { Pivot.angleSetpoint += 1.0.degrees }
//        ({ driverController.dPad == Controller.Direction.DOWN }).whenTrue { Pivot.angleSetpoint += 1.0.degrees }
//        ({ operatorController.dPad == Controller.Direction.RIGHT}).whenTrue { println("hiing"); println("end of hiing"); AutoChooser.hii() }
//        ({ operatorController.dPad == Controller.Direction.LEFT}).whenTrue { println("byeing"); AutoChooser.bye() }

        operatorController::start.whenTrue { Drive.frontSpeakerResetOdom() }



        GlobalScope.launch {
            periodic {
                // Driver Rumble
                if (Robot.isTeleopEnabled && (Shooter.motorRpmTop - Shooter.rpmTopSetpoint).absoluteValue + (Shooter.motorRpmBottom - Shooter.rpmBottomSetpoint).absoluteValue < 500.0 && Shooter.rpmTopSetpoint + Shooter.rpmBottomSetpoint > 20.0) {
                    driverController.rumble = 1.0
                } else if (Robot.isTeleopEnabled && (Intake.intakeMotorTop.output > 0.0 || Intake.intakeMotorBottom.output > 0.0)) {
                    driverController.rumble = 0.7
                } else {
                    driverController.rumble = 0.0
                }

//                println("driveRotation $driveRotation driveTranslation ${driveTranslation}")

                // Operator Rumble
                if (Shooter.manualShootState && Robot.isTeleopEnabled) {
                    operatorController.rumble = 1.0
                } else {
                    operatorController.rumble = 0.0
                }
            }
        }
    }
}
