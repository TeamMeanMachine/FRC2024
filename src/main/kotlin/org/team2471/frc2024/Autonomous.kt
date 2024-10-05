package org.team2471.frc2024

import com.choreo.lib.*;
import edu.wpi.first.math.controller.PIDController
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
        addOption("4CloseAndMid", "4CloseAndMid")
        addOption("yay", "yay")
        addOption("SubSide", "SubSide")
        addOption("FunSubSide", "FunSubSide")
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
            "4CloseAndMid" -> fourCloseAndMid()
            "yay" -> customFollowAuto()
            "SubSide" -> subSide()
            "FunSubSide" -> funSubSide()
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


        var path = Choreo.getTrajectory("4Close.1")
        println("before first execute ${Robot.totalTimeTaken()}")
        driveAlongChoreoPath(path, Drive.isRedAlliance, true)
        path = Choreo.getTrajectory("4Close.2")
        println("after first execute ${Robot.totalTimeTaken()}")
        driveAlongChoreoPath(path, Drive.isRedAlliance)
        path = Choreo.getTrajectory("4Close.3")
        driveAlongChoreoPath(path, Drive.isRedAlliance)

        println("finished all ${Robot.totalTimeTaken()}")
    }

    suspend fun fourCloseAndMid() = use(Drive, Pivot, name = "fourCloseAndMid") {
        println("inside fourCloseAndMid ${Robot.totalTimeTaken()}")
        fourClose()
        Shooter.setRpms(0.0)
        Intake.intakeState = Intake.IntakeState.INTAKING
        Pivot.angleSetpoint = Intake.intakeAngle

        val rampearlyExit: (Double) -> Boolean = {
            if (it > 0.75) {
                Shooter.setRpms(5000.0)
            }
            false
        }
        val noteEarlyExit: (Double) -> Boolean = {
            it > 0.25 && NoteDetector.closestNoteIsAtPosition(Drive.position, 10.0)
        }


        driveAlongChoreoPath(Choreo.getTrajectory("4Close.4"), Drive.isRedAlliance, earlyExit = noteEarlyExit)
        if (!Intake.holdingCargo) {
            pickUpSeenNote(ignoreWrongSide = true)
        }
        driveAlongChoreoPath(Choreo.getTrajectory("4Close.5"), Drive.isRedAlliance, earlyExit = rampearlyExit)



        Pivot.angleSetpoint = Pivot.getAngleFromPosition(Drive.position) + 2.0.degrees

        suspendUntil { /*(Shooter.averageRpm > 4500.0 && Pivot.pivotError < 0.5) || */Robot.totalTimeTaken() > 14.0}
        println("shooting at ${Robot.totalTimeTaken()}")

        fire()
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

suspend fun customFollowAuto() = use(Drive, name = "customFollowAuto") {
    println("inside customFollowAuto")
    var path = Choreo.getTrajectory("8Foot")

    driveAlongChoreoPath(path, Drive.isRedAlliance, true)
}

suspend fun subSide() = use(Drive, name = "SubSide") {
    println("Inside subSide")
    Shooter.setRpms(5000.0)
    var path = Choreo.getTrajectory("SubSide.1")

    Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE

    suspendUntil { Shooter.averageRpm > 3500.0 && Pivot.pivotError < 1.0}
    println("shooter revved ${Robot.totalTimeTaken()}")

    Intake.intakeState = Intake.IntakeState.SHOOTING

    delay(0.3.seconds)

    Shooter.setRpms(0.0)


    val rampEarlyExit: (Double) -> Boolean = {
        if (it > 0.5) {
            Shooter.setRpms(5000.0)
            Pivot.angleSetpoint = Pivot.getAngleFromPosition(AprilTag.position.asFeet)
        }
        false
    }

    val noteEarlyExit: (Double) -> Boolean = {
        it > 0.25 && NoteDetector.closestNoteIsAtPosition(Drive.position, 10.0)
    }
    Intake.intakeState = Intake.IntakeState.INTAKING

    driveAlongChoreoPath(path, Drive.isRedAlliance, true, earlyExit = noteEarlyExit)
    if (!Intake.holdingCargo) {
        pickUpSeenNote(ignoreWrongSide = true)
    }

    path = Choreo.getTrajectory("SubSide.2")
    driveAlongChoreoPath(path, Drive.isRedAlliance, earlyExit = rampEarlyExit)



    aimAndShoot()
//    Drive.position = AprilTag.position.asFeet
    Intake.intakeState = Intake.IntakeState.INTAKING

    path = Choreo.getTrajectory("SubSide.3")
    driveAlongChoreoPath(path, Drive.isRedAlliance, earlyExit = noteEarlyExit)

    if (!Intake.holdingCargo) {
        pickUpSeenNote(ignoreWrongSide = true)
    }

    path = Choreo.getTrajectory("SubSide.4")
    driveAlongChoreoPath(path, Drive.isRedAlliance, earlyExit = rampEarlyExit)

    aimAndShoot()
    Shooter.setRpms(0.0)
}

suspend fun funSubSide() = use(Drive, name = "FunSubSide") {
    println("Inside funSubSide")
    Shooter.setRpms(5000.0)
    Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE
    var path = Choreo.getTrajectory("FunSubSideIntake.1")
    suspendUntil { Shooter.averageRpm > 3000.0 }
    println("shooter revved ${Robot.totalTimeTaken()}")

    Intake.intakeState = Intake.IntakeState.SHOOTING

    delay(0.3.seconds)
    Intake.intakeState = Intake.IntakeState.INTAKING
    Shooter.setRpms(0.0)

    val noteEarlyExit: (Double) -> Boolean = {
        it > 0.25 && NoteDetector.closestNoteIsAtPosition(Drive.position, 10.0)
    }
    val rampEarlyExit: (Double) -> Boolean = {
        if (it > 0.10) {
            Shooter.setRpms(5000.0)
            Pivot.angleSetpoint = Pivot.getAngleFromPosition(AprilTag.position.asFeet)
        }
        false
    }

    println("driving \"FunSubSideIntake.1\"")
    driveAlongChoreoPath(path, Drive.isRedAlliance, earlyExit = noteEarlyExit, resetOdometry = true)

    if (!Intake.holdingCargo) {
        pickUpSeenNote(ignoreWrongSide = true)
    }

    val odomFudge = 1.0

    var pathString = ""

    pathString = if (Drive.position.y < 5.219 + odomFudge) {
        "FunSubSideFire.1"
    } else if (Drive.position.y > 5.219 + 5.505 + odomFudge) {
        "FunSubSideFire2.1"
    } else {
        "FunSubSideFire3.1"
    }

    path = Choreo.getTrajectory(pathString)
    println("returning path $pathString")
    driveAlongChoreoPath(path, Drive.isRedAlliance, earlyExit = rampEarlyExit, resetOdometry = Intake.holdingCargo)

    aimAndShoot()
    Shooter.setRpms(0.0)

    Intake.intakeState = Intake.IntakeState.INTAKING

    path = Choreo.getTrajectory("FunSubSideShooter")
    println("driving \"FunSubSideShooter\"")
    driveAlongChoreoPath(path, Drive.isRedAlliance, earlyExit = noteEarlyExit)

    pathString = if (Drive.position.y > 5.219 + 5.505 + odomFudge) {
        "FunSubSideFire2.1"
    } else {
        "FunSubSideFire3.1"
    }

    path = Choreo.getTrajectory(pathString)
    println("returning2 path $pathString")
    driveAlongChoreoPath(path, Drive.isRedAlliance, earlyExit = rampEarlyExit, resetOdometry = Intake.holdingCargo)

    aimAndShoot()
    Shooter.setRpms(0.0)
}