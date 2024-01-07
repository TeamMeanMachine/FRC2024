package org.team2471.bunnybots2023

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PneumaticHub
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.TalonID
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.util.Timer

object Intake : Subsystem("Intake") {
    val table = NetworkTableInstance.getDefault().getTable("Intake")

    val isDownEntry = table.getEntry("isDown")
    val frontPowerEntry = table.getEntry("Front Power")
    val centerPowerEntry = table.getEntry("Center Power")
    val frontMotorCurrentEntry = table.getEntry("Front Current")
    val centerMotorCurrentEntry = table.getEntry("Center Current")
    val conveyorMotorCurrentEntry = table.getEntry("Conveyor Current")
    val intakingEntry = table.getEntry("Intaking")
    val ballLoadedEntry = table.getEntry("Ball Loaded")
    val disableConveyorEntry = table.getEntry("Disabled Conveyor")
    val conveyorCurrentEntry = table.getEntry("Conveyor Current")
    val ballPastEntry = table.getEntry("Ball Past")
    val detectedBallEntry = table.getEntry("Dectected Ball")


    val frontMotor = MotorController(TalonID(Talons.INTAKE_FRONT_LEADER), TalonID(Talons.INTAKE_FRONT_FOLLOWER))
    val centerMotor = MotorController(TalonID(Talons.INTAKE_CENTER_LEFT), TalonID(Talons.INTAKE_CENTER_RIGHT))
    val conveyorMotor = MotorController(TalonID(Talons.HOPPER_CONVEYOR))
    val lowSensor = DigitalInput(DigitalSensors.HOPPER_LOW)
    val pneumaticHub = PneumaticHub(OtherCAN.PNEUMATIC_HUB)
    val solenoid = pneumaticHub.makeSolenoid(Solenoids.INTAKE)


    val ballLoaded: Boolean
        get() {return lowSensor.get()}

    val intaking: Boolean
        get() = frontMotor.current > 1.0
    val isDown = solenoid.get()

    val frontPower: Double
        get() = frontPowerEntry.getDouble(0.5).coerceIn(0.0, 1.0)
    val centerPower: Double
        get() = centerPowerEntry.getDouble(0.3).coerceIn(0.0, 1.0)

    var disableConveyor = false
    var detectedBall = false
    var ballPast = false




    init {

        frontMotor.config {
           currentLimit(15, 20,0)
           inverted(true)
           followersInverted(true)
//           coastMode()
        }
        centerMotor.config {
            currentLimit(10, 10, 0)
//            coastMode()
        }
        conveyorMotor.config {
            currentLimit(10, 0, 0)
            brakeMode()
        }

        if (!frontPowerEntry.exists()) {
           frontPowerEntry.setDouble(0.5)
           frontPowerEntry.setPersistent()
        }
        if (!centerPowerEntry.exists()) {
            centerPowerEntry.setDouble(0.3)
            centerPowerEntry.setPersistent()
        }

        GlobalScope.launch {
            periodic {
                isDownEntry.setBoolean(isDown)
                frontMotorCurrentEntry.setDouble(frontMotor.current)
                centerMotorCurrentEntry.setDouble(centerMotor.current)
                conveyorMotorCurrentEntry.setDouble(conveyorMotor.current)
                intakingEntry.setBoolean(intaking)
                ballLoadedEntry.setBoolean(ballLoaded)
                disableConveyorEntry.setBoolean(disableConveyor)
                conveyorCurrentEntry.setDouble(conveyorMotor.current)
                ballPastEntry.setBoolean(ballPast)
                detectedBallEntry.setBoolean(detectedBall)
//                println("low ${lowSensor.get()}")
                pneumaticHub.enableCompressorDigital()
            }
        }
    }

    fun startIntake() {
        frontMotor.setPercentOutput(centerPower)
        centerMotor.setPercentOutput(frontPower)
        println("Intake starting")
        if (!DriverStation.isAutonomous()) {
            OI.driverController.rumble = 0.5
        }

//        intaking = true
    }
    fun stopIntake() {
        frontMotor.setPercentOutput(0.0)
        centerMotor.setPercentOutput(0.0)
        println("Intake stopping")
        OI.driverController.rumble = 0.0
//        intaking = false
    }

    fun toggleIntake() {
        if (DriverStation.isEnabled()) {
            if (intaking) {
                stopIntake()
            } else {
                startIntake()
            }
        } else {
            println("enable before intaking")
        }
    }

    fun intakeDown() {
        solenoid.set(true)
        print("Intake Down")
    }
    fun intakeUp() {
        solenoid.set(false)
        print("Intake Up")
    }

    override fun preEnable() {
        stopIntake()
        conveyorMotor.setPercentOutput(0.0)
    }
    override fun onDisable() {
        stopIntake()
        OI.driverController.rumble = 0.0
        conveyorMotor.setPercentOutput(0.0)
        OI.driverController.rumble = 0.0
    }

    override suspend fun default() {
        var startTime = 0.0
        var startTimer = false
        val t = Timer()
        t.start()
        periodic(period = 0.005) {
            if (!disableConveyor) {
                if (Shooter.ballReady) {
                    if (startTimer) {
                        if (startTime == 0.0) {
                            startTime = t.get()
                            println("resetting start time to t.get()")
                        }
                        if (t.get() - startTime > 0.15) {
                            conveyorMotor.setPercentOutput(-0.3)
//                        println("waited, running backwards")
                            if (lowSensor.get()) {
                                conveyorMotor.setPercentOutput(0.0)
                                startTimer = false
                                println("sensor triggered. stopping loop")
                                detectedBall = true
                            }
                        }
                        if (t.get() - startTime > 1.0) {
                            println("loop took too long, resetting")
                            startTime = 0.0
                            startTimer = false
                            detectedBall = false
                        }

                    } /*else if (lowSensor.get() && !detectedBall && Shooter.ballReady) {
                    detectedBall = true
                    println("setting detected ball to true")
                }*/
                    if (lowSensor.get() && !startTimer && !detectedBall) {
                        println("detected ball, starting timer")
                        conveyorMotor.setPercentOutput(0.0)
                        startTimer = true
                        startTime = 0.0
                    } else if (!startTimer && !detectedBall) {
                        conveyorMotor.setPercentOutput(1.0)
                    }
                } else {
                    conveyorMotor.setPercentOutput(1.0)
                    startTimer = false
                    startTime = 0.0
                    detectedBall = false
                    if (lowSensor.get()) {
                        ballPast = true
                        println("ball past sensor when ball not ready, running uptake")
                    }
                }
            } else {
                conveyorMotor.setPercentOutput(0.0)
                startTime = 0.0
                startTimer = false
                ballPast = false
            }

        }
    }
}