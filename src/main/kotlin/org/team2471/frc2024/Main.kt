@file:JvmName("Main")

package org.team2471.frc2024

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.MeanlibRobot
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.units.degrees
import java.net.NetworkInterface


@DelicateCoroutinesApi
object Robot : MeanlibRobot() {
    var startMeasureTime = getSystemTimeSeconds()
    var lastMeasureTime = startMeasureTime
    var isCompBot = true

    val inComp = false

    init {
        val networkInterfaces =  NetworkInterface.getNetworkInterfaces()
        println("retrieving network interfaces")
        for (iFace in networkInterfaces) {
            println(iFace.name)
            if (iFace.name == "eth0") {
                println("NETWORK NAME--->${iFace.name}<----")
                var macString = ""
                for (byteVal in iFace.hardwareAddress){
                    macString += String.format("%s", byteVal)
                }
                println("FORMATTED---->$macString<-----")

                isCompBot = (macString != "0-1284751573")
                println("I am compbot = $isCompBot")
            }
        }

        // i heard the first string + double concatenations were expensive...
//        repeat(25) {
//            println("RANDOM NUMBER: ${Math.random()}")
//        }
        println("NEVER GONNA GIVE YOU UP")

        OI
        println("Activating OI! ${OI.driverController.leftThumbstickX}")
        Drive
        Drive.zeroGyro()
        Drive.heading = 0.0.degrees
        println("Activating Drive! heading = ${Drive.heading}")
        Intake
        println("Activating Intake! bottomBreak = ${Intake.bottomBreak}")
        Shooter
        println("Activating Shooter! rpmError = ${(Shooter.rpmTopSetpoint + Shooter.rpmBottomSetpoint) - (Shooter.motorRpmTop + Shooter.motorRpmBottom)}")
        Climb
        println("Activating Climb! climberHeight = ${Climb.climberHeight}")
        Pivot
        println("Activating Pivot! pivotEncoderAngle = ${Pivot.pivotEncoderAngle}")
        AutoChooser
        println("Activating AutoChooser! redSide = ${AutoChooser.redSide}")
        AprilTag
        println("Activating Apriltag! backCamsConnected = ${AprilTag.backCamsConnected}")
        NoteDetector
        println("Activating NoteDetector! noteCam isConnected = ${NoteDetector.camera.isConnected}")
        Limelight
        println("Activating Limelight! limelight isConnected = ${Limelight.isConnected}")

        // drop down menu for selecting tests
        val testChooser = SendableChooser<String?>().apply {
            setDefaultOption("None", null)
            addOption("Drive Tests", "Drive Tests")
        }
        SmartDashboard.putData("RobotTests", testChooser)
    }

    override suspend fun enable() {
        initTimeMeasurement()
        println("starting enable")
        var done = false
        GlobalScope.launch {
            parallel(
                {Drive.enable(); println("after drive ${totalTimeTaken()}")},
                {Shooter.enable(); println("after shooter ${totalTimeTaken()}")},
                {Climb.enable(); println("after climb ${totalTimeTaken()}")},
                {Intake.enable(); println("after intake ${totalTimeTaken()}")},
                {Pivot.enable(); println("after pivot ${totalTimeTaken()}")},
                {AprilTag.backgroundReset(); println("after aprilTag ${totalTimeTaken()}")}

            )
            done = true
        }
        suspendUntil { done }
//        println("field centric? ${SmartDashboard.getBoolean("Use Gyro", true) && !DriverStation.isAutonomous()}")
        println("ending enable ${totalTimeTaken()}")
    }

    override suspend fun autonomous() {
        println("autonomous starting")
        if (!Drive.demoMode) {
            initTimeMeasurement()
//            Drive.brakeMode()  seems to be unneeded as it is in Drive postEnable
//            Drive.aimPDController = Drive.autoPDController
            println("autonomous Drive brakeMode ${totalTimeTaken()}")
            AutoChooser.autonomous()
            println("autonomous ending ${totalTimeTaken()}")
        } else {
            println("CANNOT RUN AUTO IN DEMO MODE!!!!!! (you're welcome for not killing anyone)")
        }
    }

    override suspend fun teleop() {
        println("telop begin")
        Drive.aimPDController = Drive.teleopPDController
        Drive.headingSetpoint = Drive.heading
    }

    override suspend fun test()  {
        println("test mode begin. Hi.")

//        Drive.driveTests()
//        Drive.steeringTests()
//        Pivot.feedForwardTest()
        Drive.setAngleOffsets()
        println("test mode done")
    }


    override suspend fun disable() {
        Shooter.manualShootState = false
        Intake.intakeState = Intake.IntakeState.EMPTY
        Drive.disable()
        Climb.disable()
        Intake.disable()
        Pivot.disable()
        Shooter.disable()

        OI.driverController.rumble = 0.0
        OI.operatorController.rumble = 0.0
    }

    fun getSystemTimeSeconds(): Double {
        return System.currentTimeMillis() / 1000.0
    }

    private fun initTimeMeasurement() {
        startMeasureTime = getSystemTimeSeconds()
        lastMeasureTime = startMeasureTime
    }

    private fun updateSecondsTaken() {
        lastMeasureTime = getSystemTimeSeconds()
    }

    fun totalTimeTaken(): Double {
        return getSystemTimeSeconds() - startMeasureTime
    }

    fun recentTimeTaken(): Double {
        val timeTaken = getSystemTimeSeconds() - lastMeasureTime
        updateSecondsTaken()
        return timeTaken.toDouble()
    }
}

@OptIn(DelicateCoroutinesApi::class)
fun main() {
    println("start robot")
    RobotBase.startRobot { Robot }
}
