package org.team2471.bunnybots2023

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.TalonID
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem


object Shooter : Subsystem("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    val ballReadyEntry = table.getEntry("Ball Ready")
    val uptakeCurrentEntry = table.getEntry("Uptake Current")
    val disableUptakeEntry = table.getEntry("Disabled Uptake")
    val shooterCurrentEntry = table.getEntry("Shooter Current")
    val shooterIdleEntry = table.getEntry("Shooter Idle Power")
    val timeAfterLastShotEntry = table.getEntry("Time After Last Shot")

    val shooterMotor = MotorController(TalonID(Talons.SHOOTER_ONE), TalonID(Talons.SHOOTER_TWO))
//    val shooterMotorTwo = MotorController(TalonID(Talons.SHOOTER_TWO))
//    val shooterEncoder = AnalogInput(AnalogSensors.SHOOTER_ENCODER)
    val uptakeMotor = MotorController(TalonID(Talons.HOPPER_UPTAKE))
    val uptakeSensor = DigitalInput(DigitalSensors.HOPPER_HIGH)

    var lastShotTime: Double = Timer.getFPGATimestamp()
    val timeSinceLastShot: Double
        get() = Timer.getFPGATimestamp() - lastShotTime


    var ballReady: Boolean = !uptakeSensor.get()
        get() { /*prevBallReady = field*/; field = !uptakeSensor.get(); return field }
//    var prevBallReady = ballReady

    var disableUptake = false
    var detectedBall = false
    var reverseBall = false
    var timeAfterLastShot = 1.0
        get() = field + DriverStation.getMatchTime()
        set(value) { field = DriverStation.getMatchTime() + value }

    val shooterIdlePower: Double
        get() = 1.0//shooterIdleEntry.getDouble(0.85).coerceIn(0.0, 1.0)

    init {

        uptakeMotor.config {
            currentLimit(20, 0, 0)
            inverted(true)
            brakeMode()
        }
        shooterMotor.config {
            currentLimit(15, 20, 0)
            inverted(true)
            coastMode()
            followersInverted(false) //DO NOT MAKE THIS VALUE TRUE
            openLoopRamp(1.5)
        }
        if (!shooterIdleEntry.exists()) {
            shooterIdleEntry.setPersistent()
            shooterIdleEntry.setDouble(0.85)
        }


        GlobalScope.launch {
            periodic {
//                println("high ${uptakeSensor.get()}")
                ballReadyEntry.setBoolean(ballReady)
                uptakeCurrentEntry.setDouble(uptakeMotor.current)
                disableUptakeEntry.setBoolean(disableUptake)
                shooterCurrentEntry.setDouble(shooterMotor.current)
                timeAfterLastShotEntry.setDouble(timeAfterLastShot)
                if (DriverStation.isEnabled() && ballReady && Limelight.seesTargets) {
                    if (timeSinceLastShot > 1.0) {
                        OI.operatorController.rumble = 0.25
                    } else {
                        OI.operatorController.rumble = 0.1
                    }
                } else {
                    OI.operatorController.rumble = 0.0
                }
            }
        }
    }

    override fun onDisable() {
        OI.operatorController.rumble = 0.0
    }

    override suspend fun default() {
        println("Shooter: Starting default")
        val t = Timer()
        var waitTime = 0.0
        var waiting = false
        t.start()
        periodic(period = 0.005) {
            if (!disableUptake) {
                if (ballReady) {
                    uptakeMotor.setPercentOutput(0.0)
                    Intake.ballPast = false
                    waitTime = t.get()
                } else if (Intake.ballPast) {
                    uptakeMotor.setPercentOutput(1.0)
                } else if (t.get() - waitTime > 1.2) {
                    uptakeMotor.setPercentOutput(0.0)
                } else {
                    uptakeMotor.setPercentOutput(1.0)
                }
                shooterMotor.setPercentOutput(1.0)

            } else {
                uptakeMotor.setPercentOutput(0.0)
                shooterMotor.setPercentOutput(0.0)
            }

        }
    }

    fun setLastShotTime() {
        lastShotTime = Timer.getFPGATimestamp()
    }
}