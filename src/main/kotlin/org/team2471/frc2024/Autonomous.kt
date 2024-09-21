package org.team2471.frc2024

import com.choreo.lib.*;
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PIDSubsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.asFeet
import org.team2471.frc.lib.math.feet
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion.following.xPose
import org.team2471.frc.lib.motion_profiling.Autonomi
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.testing.*
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.Timer
import org.team2471.frc.lib.util.isReal
import org.team2471.frc.lib.util.measureTimeFPGA
import org.team2471.frc2024.Drive.isBlueAlliance
import org.team2471.frc2024.Drive.isRedAlliance
import org.team2471.frc2024.gyro.GyroIO
import java.io.File
import java.util.*
import kotlin.math.absoluteValue


val selAuto
    get() = SmartDashboard.getString("AutoChooser/selected", "no auto selected")

object AutoChooser {

    private val lyricsChooser = SendableChooser<String?>().apply {
        setDefaultOption("Country roads", "Country roads")
        addOption("take me home", "take me home")
    }

    private val autoChooser = SendableChooser<String?>().apply {
        addOption("None", null)
        addOption("TestAuto", "TestAuto")
    }

    init {

        SmartDashboard.putData("AutoChooser", autoChooser)

        SmartDashboard.putData("Best Song Lyrics", lyricsChooser)
        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                SmartDashboard.updateValues()

                val autoChosen = selAuto == "no auto selected" || selAuto != "Tests" || selAuto == ""

                SmartDashboard.putBoolean("Auto is selected", autoChosen)
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
            else -> println("No function found for ---->$selAuto<-----  ${Robot.totalTimeTaken()}")
        }
        SmartDashboard.putString("autoStatus", "complete")
        println("finished autonomous  ${Robot.totalTimeTaken()}")
    }

    suspend fun testAuto() = use(Drive, name = "Test Auto") {

        val trajectory = Choreo.getTrajectory("NewPath")

        val initalPose = trajectory.initialPose

        Drive.position = Vector2(initalPose.x.meters.asFeet, initalPose.y.meters.asFeet)

        Drive.heading = initalPose.rotation.asAngle

        println("Trajectory:    $trajectory")

        val autoCommand = Choreo.choreoSwerveCommand(
            trajectory,
            Drive::getPose,
            PIDController(Drive.parameters.kpPosition.feet.asMeters, 0.0, Drive.parameters.kdPosition.feet.asMeters),
            PIDController(Drive.parameters.kpPosition.feet.asMeters, 0.0, Drive.parameters.kdPosition.feet.asMeters),
            PIDController(Drive.parameters.kpHeading.feet.asMeters, 0.0, Drive.parameters.kdHeading.feet.asMeters),
            Drive::driveRobotRelative,
            Drive::isBlueAlliance,
            object : SubsystemBase() {}
        )

        var interrupted = true

        autoCommand.initialize()

        println("Right before periodic")
        periodic {
            autoCommand.execute()

            if (autoCommand.isFinished) {
                interrupted = false
                println("Finished!")
                this.stop()
            }
        }
        autoCommand.end(interrupted)
    }
}

/*
    suspend fun pathPlannerAuto() = use(Drive, name = "Path Planner Auto")
    {
        println("Entered pathPlanner Auto. Getting command")
        val autoCommand = */
/*PathPlannerAuto("4Close")*//*
 autoChooser.selected
//        val autoCommand = PathPlannerAuto(tempCommand.name)

        println("Got command: ${autoCommand.name}")

        var interrupted = true

        println("Initializing...")
        autoCommand.initialize()

        println("Right before periodic")
        periodic {
            autoCommand.execute()

            if (autoCommand.isFinished) {
                interrupted = false
                println("Finished!")
                this.stop()
            }
        }
        autoCommand.end(interrupted)
    }
}*/
