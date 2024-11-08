package org.team2471.frc2024

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.asFeet
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.Timer
import kotlin.io.path.listDirectoryEntries
import kotlin.io.path.name

object AutoChooser {

    val selAuto
        get() = SmartDashboard.getString("AutoChooser/selected", "no auto selected")

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

    //load choreo paths
    val paths: MutableMap<String, ChoreoTrajectory?> = mutableMapOf(
        *Filesystem.getDeployDirectory().toPath().resolve("choreo").listDirectoryEntries("*.traj").map {
            val name = it.name.removeSuffix(".traj")
            Pair(name, Choreo.getTrajectory(name))
        }.toTypedArray()
    )

    init {
        println("loaded paths: ${paths.map { Pair(it.key, it.value?.samples?.size).toString() }}")

        SmartDashboard.putData("AutoChooser", autoChooser)
        SmartDashboard.putData("Best Song Lyrics", lyricsChooser)

        var isPathRed = false //all paths start blue
        var prevPathRed: Boolean? = null

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                SmartDashboard.updateValues()

                val autoChosen = selAuto == "no auto selected" || selAuto != "Tests" || selAuto == ""
                SmartDashboard.putBoolean("Auto is selected", autoChosen)

                val isRed = Drive.isRedAlliance
                if (prevPathRed != null) {
                    if (prevPathRed != isRed) {
                        flipPaths()
                        isPathRed = !isPathRed
                    }
                } else {
                    if (isRed) {
                        flipPaths()
                        isPathRed = !isPathRed
                    }
                }
                prevPathRed = isPathRed
            }
        }
    }

    private fun flipPaths() = paths.replaceAll { _, t -> t?.flipped() }

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
            "SubSide" -> subSide()
            "FunSubSide" -> funSubSide()
            else -> println("No function found for ---->$selAuto<-----  ${Robot.totalTimeTaken()}")
        }
        SmartDashboard.putString("autoStatus", "complete")
        println("finished autonomous  ${Robot.totalTimeTaken()}")
    }

    suspend fun testAuto() = use(Drive, name = "Test Auto") {

        val trajectory = paths["8Foot"]

        if (trajectory != null) {
            driveAlongChoreoPath(trajectory, true)
        } else {
            println("path is null")
        }
    }

    suspend fun fourClose() = use(Drive, Shooter, Pivot, name = "Four Close") {
        println("inside fourClose ${Robot.totalTimeTaken()}")
        Shooter.setRpms(5000.0)
        Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE

        val t = Timer()
        t.start()

        suspendUntil { Shooter.averageRpm > 3000.0 }
        println("shooter revved ${Robot.totalTimeTaken()}")

        Intake.intakeState = Intake.IntakeState.SHOOTING

        delay(0.3.seconds)


        Pivot.angleSetpoint = Pivot.PODIUMPOSE - 3.0.degrees


        var path = paths["4Close.1"]
        println("before first execute ${Robot.totalTimeTaken()}")
        if (path != null) driveAlongChoreoPath(path, true)
        path = paths["4Close.2"]
        println("after first execute ${Robot.totalTimeTaken()}")
        if (path != null) driveAlongChoreoPath(path)
        path = paths["4Close.3"]
        if (path != null) driveAlongChoreoPath(path)

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


        paths["4Close.4"]?.let { driveAlongChoreoPath(it, earlyExit = noteEarlyExit) }

        if (!Intake.holdingCargo) {
            pickUpSeenNote(ignoreWrongSide = true)
        }

        paths["4Close.5"]?.let { driveAlongChoreoPath(it, earlyExit = rampearlyExit) }


        aimAndShoot(false, minTime = 2.0, 0.0, 14.0)

//        Pivot.angleSetpoint = Pivot.getAngleFromPosition(Drive.position) + 2.0.degrees

//        suspendUntil { /*(Shooter.averageRpm > 4500.0 && Pivot.pivotError < 0.5) || */Robot.totalTimeTaken() > 14.0}
        println("shot at ${Robot.totalTimeTaken()}")
    }

    suspend fun subSide() = use(Drive, name = "SubSide") {
        println("Inside subSide")
        Shooter.setRpms(5000.0)
        Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE

        var path = paths["SubSide.1"]
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

        suspendUntil { Shooter.averageRpm > 3500.0 && Pivot.pivotError < 1.0 }
        println("shooter revved ${Robot.totalTimeTaken()}")

        Intake.intakeState = Intake.IntakeState.SHOOTING

        delay(0.3.seconds)

        Shooter.setRpms(0.0)

        Intake.intakeState = Intake.IntakeState.INTAKING

        if (path != null) {
            driveAlongChoreoPath(path, true, earlyExit = noteEarlyExit)
        }

        if (!Intake.holdingCargo) {
            pickUpSeenNote(ignoreWrongSide = true)
        }

        path = paths["SubSide.2"]
        if (path != null) {
            driveAlongChoreoPath(path, earlyExit = rampEarlyExit)
        }



        aimAndShoot()
//    Drive.position = AprilTag.position.asFeet
        Intake.intakeState = Intake.IntakeState.INTAKING

        path = paths["SubSide.3"]
        if (path != null) {
            driveAlongChoreoPath(path, earlyExit = noteEarlyExit)
        }

        if (!Intake.holdingCargo) {
            pickUpSeenNote(ignoreWrongSide = true)
        }

        path = paths["SubSide.4"]
        if (path != null) {
            driveAlongChoreoPath(path, earlyExit = rampEarlyExit)
        }

        aimAndShoot()
        Shooter.setRpms(0.0)
    }

    suspend fun funSubSide() = use(Drive, name = "FunSubSide") {
        println("Inside funSubSide")
        Shooter.setRpms(5000.0)
        Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE
        var path = paths["FunSubSideIntake.1"]
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
        if (path != null) {
            driveAlongChoreoPath(path, earlyExit = noteEarlyExit, resetOdometry = true)
        }

        if (!Intake.holdingCargo) {
            pickUpSeenNote(ignoreWrongSide = true)
        }

        val odomFudge = 1.0

        var pathString = if (Drive.position.y < 5.219 + odomFudge) {
            "FunSubSideFire.1"
        } else if (Drive.position.y > 5.219 + 5.505 + odomFudge) {
            "FunSubSideFire2.1"
        } else {
            "FunSubSideFire3.1"
        }

        path = paths[pathString]
        println("returning path $pathString")
        if (path != null) {
            driveAlongChoreoPath(path, earlyExit = rampEarlyExit, resetOdometry = Intake.holdingCargo)
        }

        aimAndShoot()
        Shooter.setRpms(0.0)

        Intake.intakeState = Intake.IntakeState.INTAKING

        path = paths["FunSubSideShooter"]
        println("driving \"FunSubSideShooter\"")
        if (path != null) {
            driveAlongChoreoPath(path, earlyExit = noteEarlyExit)
        }

        pathString = if (Drive.position.y > 5.219 + 5.505 + odomFudge) {
            "FunSubSideFire2.1"
        } else {
            "FunSubSideFire3.1"
        }

        path = paths[pathString]
        println("returning2 path $pathString")
        if (path != null) {
            driveAlongChoreoPath(path, earlyExit = rampEarlyExit, resetOdometry = Intake.holdingCargo)
        }

        aimAndShoot()
        Shooter.setRpms(0.0)
    }
}