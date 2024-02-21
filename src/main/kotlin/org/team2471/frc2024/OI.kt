package org.team2471.frc2024

import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.input.*
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.squareWithSign
import org.team2471.frc.lib.motion.following.xPose
import org.team2471.frc.lib.units.degrees

object OI : Subsystem("OI") {
    val driverController = XboxController(0)
    val operatorController = XboxController(1)

    private val deadBandDriver = 0.05
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
        driverController::x.whenTrue { Drive.xPose() }

        driverController::leftBumper.whenTrue { Intake.intaking = !Intake.intaking }
        driverController::rightBumper.whenTrue { spit() }
        driverController::rightTriggerFullPress.whenTrue { fire() }
        driverController::a.whenTrue { Shooter.shootingRPM = !Shooter.shootingRPM }
//        driverController::b.whenTrue { aimAtSpeaker() }
//        driverController::b.whenTrue { pickUpSeenNote() }

        ({ driverController.dPad == Controller.Direction.LEFT}).whenTrue { Pivot.angleSetpoint += 1.degrees }
        ({ driverController.dPad == Controller.Direction.RIGHT}).whenTrue { Pivot.angleSetpoint -= 1.degrees }



        operatorController::y.whenTrue { Pivot.angleSetpoint = Pivot.MAXHARDSTOP.degrees }
        operatorController::b.whenTrue { Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE.degrees }
        operatorController::a.whenTrue { Pivot.angleSetpoint = Pivot.MINHARDSTOP.degrees + 2.0.degrees }
        operatorController::x.whenTrue { Pivot.angleSetpoint = Pivot.TESTPOSE.degrees }

        ({operatorRightTrigger > 0.03}).whenTrue { println("climbinggggggggggggggggggg"); climbWithTrigger() }

        ({operatorController.leftBumper && operatorController.rightBumper}).whenTrue { println("LOCKING NOWWWWWWWWWWWW!!!!"); Climb.activateRelay() }

        ({ operatorController.dPad == Controller.Direction.DOWN}).whenTrue { Shooter.rpmTop -= 5.0; /*Climb.climberSetpoint -= 5.0.inches*/ }
        ({ operatorController.dPad == Controller.Direction.UP}).whenTrue { Shooter.rpmTop += 5.0; /*Climb.climberSetpoint += 5.0.inches*/ }
        ({ operatorController.dPad == Controller.Direction.LEFT}).whenTrue { Shooter.rpmBottom -= 5.0; /*Climb.climberSetpoint -= 5.0.inches*/ }
        ({ operatorController.dPad == Controller.Direction.RIGHT}).whenTrue { Shooter.rpmBottom += 5.0; /*Climb.climberSetpoint -= 5.0.inches*/ }

        operatorController::start.whenTrue { resetCameras() }



    }
}
