@file:JvmName("Main")

package org.team2471.frc2024

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import org.team2471.frc.lib.framework.MeanlibRobot
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.testing.*
import java.net.NetworkInterface


@DelicateCoroutinesApi
object Robot : MeanlibRobot() {
    var startMeasureTime = System.nanoTime()
    var lastMeasureTime = startMeasureTime
    var isCompBot = true
    var beforeFirstEnable = true
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
        println("Activating Drive!")
        Drive
        Drive.zeroGyro()
        Drive.heading = 0.0.degrees
        Intake
        Shooter
        Climb
        Pivot
        AutoChooser
        AprilTag
        PoseEstimator
        NoteDetector
        println("Activating AutoChooser! redSide = ${AutoChooser.redSide}")

        // drop down menu for selecting tests
        val testChooser = SendableChooser<String?>().apply {
            setDefaultOption("None", null)
            addOption("Drive Tests", "Drive Tests")
        }
        SmartDashboard.putData("RobotTests", testChooser)
    }

    override suspend fun enable() {
        beforeFirstEnable = false
        println("starting enable")
        Drive.enable()
        Climb.enable()
        Intake.enable()
        Pivot.enable()
        Shooter.enable()
        println("field centric? ${SmartDashboard.getBoolean("Use Gyro", true) && !DriverStation.isAutonomous()}")
        println("ending enable")
    }

    override suspend fun autonomous() {
        if (!Drive.demoMode) {
            initTimeMeasurement()
            println("autonomous starting")
            Drive.brakeMode()
            Drive.aimPDController = Drive.autoPDController
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

    override suspend fun test() {
        println("test mode begin. Hi.")


        Drive.driveTests()
        Drive.steeringTests()



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

    private fun initTimeMeasurement() {
        startMeasureTime = System.nanoTime()
        lastMeasureTime = startMeasureTime
    }

    private fun updateNanosTaken() {
        lastMeasureTime = System.nanoTime()
    }

    fun totalTimeTaken(): Long {
        return System.nanoTime() - startMeasureTime
    }

    fun recentTimeTaken(): Long {
        val timeTaken = System.nanoTime() - lastMeasureTime
        updateNanosTaken()
        return timeTaken
    }
}

@OptIn(DelicateCoroutinesApi::class)
fun main() {
    println("start robot")
    RobotBase.startRobot { Robot }
}
