package org.team2471.bunnybots2023

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DriverStation
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.bunnybots2023.Limelight.limelightAngle
import org.team2471.bunnybots2023.Limelight.toFieldCentric
import org.team2471.bunnybots2023.Limelight.toRobotCentric
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.*
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.ceil

object Turret : Subsystem("Turret") {

    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    val turretErrorEntry = table.getEntry("Turret Error")
    val turretCurrentEntry = table.getEntry("Turret Current")
    val turretAngleEntry = table.getEntry("Turret Angle")
    val fieldTurretSetpointEntry = table.getEntry("Field Centric Turret Setpoint")
    val robotTurretSetpointEntry = table.getEntry("Robot Centric Turret Setpoint")
    val turretEncoderAngleEntry = table.getEntry("Turret Encoder Angle")
    val encoderVoltageEntry = table.getEntry("Raw Encoder Voltage")
    val pBotCentCoordsAngleEntry = table.getEntry("PredBotCentCoord Angle (50)")

    var ticks = 0.0
    val ticksFilter = LinearFilter.movingAverage(5)

    const val minJoystickDistance = 0.5

    // Same direction
    val turningMotor = MotorController(FalconID(Falcons.TURRET_ONE), FalconID(Falcons.TURRET_TWO))
    val turretEncoder = AnalogInput(AnalogSensors.TURRET_ENCODER)

    val turretGearRatio: Double = 4.0 * (72.0/24.0) * (70.0/12.0)//(70.0/12.0) * (64.0/32.0) * (4.0)

    // robot centric
    val maxAngle : Angle = 100.0.degrees
    val minAngle : Angle = (-230.0).degrees

    // in robot centric
    val turretAngle: Angle
        get() = turningMotor.position.degrees
    val turretEncoderAngle: Angle
        get() {
            var rawAngle = -((turretEncoder.voltage - 0.2) / 4.6 * 360.0 / (70.0/12.0) - 50.0).degrees
            var newAngle = rawAngle
            if (rawAngle > 30.0.degrees) {
                newAngle = rawAngle - 61.0.degrees
            }
//            println("rawAngle $rawAngle  newAngle $newAngle")
            return newAngle
        }

    val turretError: Angle
        get() = turretSetpoint.toRobotCentric() - turretAngle

    var rawTurretSetpoint: Angle = 0.0.degrees


    // in field centric
    var turretSetpoint: Angle = 0.0.degrees
        set(value) {

//            println("HI!!!")
            var angle = value.toRobotCentric() //+ turretSetpointOffset

            val minDist = (angle - minAngle).wrap()
            val maxDist = (angle - maxAngle).wrap()

            if (minDist.asDegrees.absoluteValue < maxDist.asDegrees.absoluteValue) {
                angle = angle.unWrap(minAngle)
            } else {
                angle = angle.unWrap(maxAngle)
            }

            angle = angle.asDegrees.coerceIn(minAngle.asDegrees, maxAngle.asDegrees).degrees
            turningMotor.setPositionSetpoint(angle.asDegrees)
            field = angle.toFieldCentric()
        }
    var turretSetpointOffset: Angle = 0.0.degrees
    var autoAim = true
    var turretPredAim = true

    init {
//        println("*******************************************************************************************************")
        turningMotor.restoreFactoryDefaults()
        turningMotor.config() {
            //                            ticks / gear ratio             fudge factor
            feedbackCoefficient = (360.0 / 2048.0 / turretGearRatio)// * (90.0/136.0)

            coastMode()
            inverted(false)
            pid {
                p(0.00000016)//p(0.0000002)
                d(0.00001)
            }
            currentLimit(30, 40, 20)

            encoderType(FeedbackDevice.IntegratedSensor)
            burnSettings()
            setRawOffsetConfig(turretEncoderAngle.asDegrees)
        }

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                turretAngleEntry.setDouble(turretAngle.asDegrees)
                turretCurrentEntry.setDouble(turningMotor.current)
                fieldTurretSetpointEntry.setDouble(turretSetpoint.asDegrees)
                robotTurretSetpointEntry.setDouble(turretSetpoint.toRobotCentric().asDegrees)
                turretErrorEntry.setDouble(turretError.asDegrees)
                turretEncoderAngleEntry.setDouble(turretEncoderAngle.asDegrees)
                encoderVoltageEntry.setDouble(turretEncoder.voltage)

                if (DriverStation.isEnabled()) {
                    turretSetpointOffset = (OI.operatorRightX).degrees
//                    println("stick = ${OI.operatorRightX}")
                }
                turretSetpoint = rawTurretSetpoint
            }
        }
    }

    override suspend fun default() {

        periodic {
            // sets joystickTarget to the current angle of the right joystick, null if at center
            val joystickTarget : Angle? = if (OI.operatorController.leftThumbstick.length > minJoystickDistance) {
                90.degrees + atan2(OI.operatorLeftY, OI.operatorLeftX).radians
            } else {
                null
            }

            // handle joystick input
            if (joystickTarget != null) {

                val upperAimingBound : Angle = joystickTarget + 20.0.degrees
                val lowerAimingBound : Angle = joystickTarget - 20.0.degrees

                val target : BucketTarget? = Limelight.getBucketInBounds(upperAimingBound, lowerAimingBound)

                if (target != null) {
                    aimAtBucket(target)

                } else {
                    rawTurretSetpoint = joystickTarget
                }

            } else if (Limelight.enemyBuckets.isNotEmpty()) {
                aimAtBucket(Limelight.enemyBuckets[0])
            } else {
                rawTurretSetpoint = rawTurretSetpoint
//                println("setpoint = $turretSetpoint")
            }



//            if (opX) {
//                println("whhhhhhhhyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy")
//                turretGO()
//            } else {
//                turretStop()
//            }
        }
    }

    override fun preEnable() {
        rawTurretSetpoint = turretAngle.toFieldCentric()
    }

    fun aimAtBucket(target : BucketTarget){
        if(autoAim) {
            if (turretPredAim) {
                ticks = target.ticksToTarget

//        println(ticks)
                rawTurretSetpoint = target.pBotCentCoords(2.0/*ticksFilter.calculate(ticks)*/).angle
//        println(angle - target.botCentCoords.angle)
//        pBotCentCoordsAngleEntry.setDouble(target.pBotCentCoords(20).angle.asDegrees)
//        println(target.vAngle)
            } else {
                rawTurretSetpoint = target.angle
            }
        }
    }

    fun turretRight() {
//        turningMotor.setPercentOutput(0.5)
//        delay(0.5)
        rawTurretSetpoint = 90.0.degrees
    }

    fun zeroTurret() {
        turningMotor.setRawOffset(0.0)
        rawTurretSetpoint = turretAngle.toFieldCentric()
    }


    fun turretLeft() {
//        turningMotor.setPercentOutput(0.0)
        rawTurretSetpoint = (-90.0).degrees
    }

}