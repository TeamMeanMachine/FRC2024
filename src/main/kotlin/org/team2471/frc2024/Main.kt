@file:JvmName("Main")

package org.team2471.frc2024

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.LoggedMeanlibRobot
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.util.RobotMode
import org.team2471.frc.lib.util.robotMode
import java.net.NetworkInterface


@DelicateCoroutinesApi
object Robot : LoggedMeanlibRobot() {
    var startMeasureTime = getSystemTimeSeconds()
    var lastMeasureTime = startMeasureTime
    val isCompBot = getCompBotBoolean()

    private val loggedComponentPosesList: HashMap<String, Pose3d> = hashMapOf()

    val inComp = false

    val subsystems: Array<Subsystem> = arrayOf(LedControl, OI, Drive, Intake, Pivot, Shooter, Climb, AprilTag)

    init {
        println("robotMode == $robotMode")
        if (robotMode != RobotMode.REPLAY) {
            //sim or real
            Logger.addDataReceiver(WPILOGWriter())
            Logger.addDataReceiver(NT4Publisher())
        } else {
            setUseTiming(true) // false = run sim as fast as possible
            val logPath = LogFileUtil.findReplayLog()
            Logger.setReplaySource(WPILOGReader(logPath))
            Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
        }

        Logger.start()

        LedControl

        // i heard the first string + double concatenations were expensive...
//        repeat(25) {
//            println("RANDOM NUMBER: ${Math.random()}")
//        }
        println("NEVER GONNA GIVE YOU UP")

        for (subsystem in subsystems) {
            println("activating subsystem ${subsystem.name}")
        }

        // drop down menu for selecting tests
        val testChooser = SendableChooser<String?>().apply {
            setDefaultOption("None", null)
            addOption("Drive Tests", "Drive Tests")
        }
        SmartDashboard.putData("RobotTests", testChooser)
        LedControl.pattern = LedPatterns.DISABLED
        GlobalScope.launch {
            periodic {
                try {
                    loggedComponentPosesList.forEach { Logger.recordOutput(it.key, it.value) }
                } catch (_: Exception) {}
            }
        }
    }

    override suspend fun enable() {
        initTimeMeasurement()
        println("starting enable")
        var done = false
        GlobalScope.launch {
            //creates a list of enable functions for each subsystem, then run them in parallel. Tested in sim and was same speed as old code -Justin
            parallel(
                *subsystems.map { suspend { it.enable(); println("after ${it.name} ${totalTimeTaken()}") } }.toTypedArray()
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

        subsystems.forEach { it.disable() }

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
        return timeTaken
    }

    private fun getCompBotBoolean(): Boolean {
        var compBot = true
        if (robotMode == RobotMode.REAL) {
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

                    compBot = (macString != "0-1284751573")
                }
            }
        } else { println("Not real so I am compbot") }
        println("I am compbot = $compBot")
        return compBot
    }
    fun logComponent(name: String, pose: Pose3d) {
        loggedComponentPosesList[name] = pose
    }
}

@OptIn(DelicateCoroutinesApi::class)
fun main() {
    println("start robot")
    RobotBase.startRobot { Robot }
}
