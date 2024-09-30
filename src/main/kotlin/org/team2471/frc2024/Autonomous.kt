package org.team2471.frc2024

import com.choreo.lib.*;
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.Odometry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.Vector2L
import org.team2471.frc.lib.math.asFeet
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.Timer
import org.team2471.frc2024.AutoChooser.initialPose


val selAuto
    get() = SmartDashboard.getString("AutoChooser/selected", "no auto selected")

object AutoChooser {

    private var prevAutoChoosen = ""
    private var prevAlliance: Boolean? = null

    var initialPose: Pair<Vector2, Angle> = Pair(Vector2(0.0, 0.0), 0.0.radians)

    private val lyricsChooser = SendableChooser<String?>().apply {
        setDefaultOption("Country roads", "Country roads")
        addOption("take me home", "take me home")
    }

    private val autoChooser = SendableChooser<String?>().apply {
        addOption("None", null)
        addOption("TestAuto", "TestAuto")
        addOption("4Close", "4Close")
    }

    private val paths: ArrayList<Command> = arrayListOf()

    init {

        SmartDashboard.putData("AutoChooser", autoChooser)

        SmartDashboard.putData("Best Song Lyrics", lyricsChooser)
        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                SmartDashboard.updateValues()

                val autoChosen = selAuto == "no auto selected" || selAuto != "Tests" || selAuto == ""


                SmartDashboard.putBoolean("Auto is selected", autoChosen)

                val isRed = Drive.isRedAlliance

                if (prevAutoChoosen != selAuto || isRed != prevAlliance) {
                    println("Autonomous change detected")
                    when (selAuto) {
                        "4Close" -> {
                            paths.addAll( arrayListOf(
                                    initializeChoreoPath("4Close.1", true),
                                    initializeChoreoPath("4Close.2"),
                                    initializeChoreoPath("4Close.3")
                            ))
                        }
                        "TestAuto" -> {
                            paths.addAll(
                                arrayListOf(
                                    initializeChoreoPath("8Foot.1", true)
                                )
                            )
                        }
                        else -> println("no paths :(")
                    }

                    prevAutoChoosen = selAuto
                    prevAlliance = isRed
                }
            }
        }
    }

    @OptIn(DelicateCoroutinesApi::class)
    suspend fun autonomous() = use(Drive, name = "Autonomous") {
        println("Got into Auto fun autonomous. Hi. 888888888888888 ${Robot.totalTimeTaken()}")
//        SmartDashboard.putString("autoStatus", "init")
        println("Selected Auto = ******* $selAuto *******  ${Robot.totalTimeTaken()}")
        println("before when block ${Robot.totalTimeTaken()}")
        when (selAuto) {
            "TestAuto" -> testAuto()
            "4Close" -> fourClose()
            else -> println("No function found for ---->$selAuto<-----  ${Robot.totalTimeTaken()}")
        }
        SmartDashboard.putString("autoStatus", "complete")
        println("finished autonomous  ${Robot.totalTimeTaken()}")
    }

    suspend fun testAuto() = use(Drive, name = "Test Auto") {

        Drive.position = initialPose.first
        Drive.heading = initialPose.second

        executeChoreoCommand(paths[0])
    }

    suspend fun fourClose() = use(Drive, Shooter, Pivot, name = "Four Close") {
        println("inside fourClose ${Robot.totalTimeTaken()}")

        val t = Timer()
        t.start()

        Drive.position = initialPose.first
        Drive.heading = initialPose.second

        Shooter.setRpms(5000.0)

        Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE

        suspendUntil { Shooter.averageRpm > 3000.0 }
        println("shooter revved ${Robot.totalTimeTaken()}")

        Intake.intakeState = Intake.IntakeState.SHOOTING

        delay(0.3.seconds)


        Pivot.angleSetpoint = Pivot.PODIUMPOSE - 4.0.degrees


        println("before first execute ${Robot.totalTimeTaken()}")
        executeChoreoCommand(paths[0])
        println("after first execute ${Robot.totalTimeTaken()}")
        executeChoreoCommand(paths[1])
        executeChoreoCommand(paths[2])

        println("finished all ${Robot.totalTimeTaken()}")
    }
}

fun initializeChoreoPath(pathName: String, resetOdometry: Boolean = false): Command {
    println("Init choreo parh $pathName")
    val trajectory = Choreo.getTrajectory(pathName)

    if (resetOdometry) {
        val initialPos = Vector2L(trajectory.initialPose.x.meters, trajectory.initialPose.y.meters)

        if (Drive.isRedAlliance) {
            initialPose = Pair(
                initialPos.asFeet.reflectAcrossField(),
                (180.0.degrees - trajectory.initialPose.rotation.asAngle).wrap() // may not be correct if not 0 or 180
            )
        } else {
            initialPose = Pair(
                initialPos.asFeet,
                trajectory.initialPose.rotation.asAngle
            )
        }
    }
    println("Trajectory:    ${Drive.getPose()}")

    return Choreo.choreoSwerveCommand(
        trajectory,
        Drive::getPose,
        PIDController(Drive.parameters.kpPosition.feet.asMeters / 20.0, 0.0, Drive.parameters.kdPosition.feet.asMeters / 20.0),
        PIDController(Drive.parameters.kpPosition.feet.asMeters / 20.0, 0.0, Drive.parameters.kdPosition.feet.asMeters / 20.0),
        PIDController(Drive.parameters.kpHeading.feet.asMeters / 20.0, 0.0, Drive.parameters.kdHeading.feet.asMeters / 20.0),
        Drive::driveRobotRelative,
        Drive::isRedAlliance,
        object : SubsystemBase() {}
    )


}

suspend fun executeChoreoCommand(autoCommand: Command) = use(Drive, name = "Choreo Path") {
    println("Init path. ${Robot.totalTimeTaken()}")
    autoCommand.initialize()// for some reason this takes 1.7 seconds
    println("Initialized path: ${Robot.totalTimeTaken()}")

    var interrupted = true


    println("Right before periodic: ${Robot.totalTimeTaken()}")
    periodic {
        autoCommand.execute()

        if (autoCommand.isFinished) {
            interrupted = false
            println("Finished! ${Robot.totalTimeTaken()}")
            this.stop()
        }
    }
    autoCommand.end(interrupted)
    println("ended ${Robot.totalTimeTaken()}")
    Drive.drive(Vector2(0.0, 0.0), 0.0)
}