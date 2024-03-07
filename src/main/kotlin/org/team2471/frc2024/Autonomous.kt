package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion_profiling.Autonomi
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.util.measureTimeFPGA
import java.io.File
import java.util.*

private lateinit var autonomi: Autonomi


enum class Side {
    LEFT,
    RIGHT;

    operator fun not(): Side = when (this) {
        LEFT -> RIGHT
        RIGHT -> LEFT
    }
}

val selAuto
    get() = SmartDashboard.getString("Autos/selected", "no auto selected")

object AutoChooser {
    private val isRedAllianceEntry = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("isRedAlliance")
    private val closeFourToFiveEntry = PoseEstimator.poseTable.getEntry("Auto Close4 to 5")
    private var autonomiEntryTopicSub =
        NetworkTableInstance.getDefault().getTable("PathVisualizer").getStringTopic("Autonomi").subscribe("")

    var cacheFile: File? = null
    var redSide: Boolean = true
        get() {
            if (DriverStation.getAlliance().isEmpty) {
//                println("DriverStation.getAlliance() = null!!!!!!!!!!!!!!!!!! defaulting to isRedAlliance to true")
                return true
            } else {
                return DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            }
        }
        set(value) {
            field = value
            isRedAllianceEntry.setBoolean(value)
        }

    private val lyricsChooser = SendableChooser<String?>().apply {
        setDefaultOption("Country roads", "Country roads")
        addOption("take me home", "take me home")
    }

    private val testAutoChooser = SendableChooser<String?>().apply {
        addOption("None", null)
        addOption("20 Foot Test", "20 Foot Test")
        addOption("8 Foot Straight", "8 Foot Straight")
        addOption("2 Foot Circle", "2 Foot Circle")
        addOption("4 Foot Circle", "4 Foot Circle")
        addOption("8 Foot Circle", "8 Foot Circle")
        addOption("Full Field Straight B","Full Field Straight B")
        addOption("Full Field Straight R","Full Field Straight R")
        addOption("Hook Path", "Hook Path")
        setDefaultOption("90 Degree Turn", "90 Degree Turn")


    }

    private val autonomousChooser = SendableChooser<String?>().apply {
        setDefaultOption("Tests", "testAuto")
        addOption("2Far2CloseAmp", "2Far2CloseAmp")
        addOption("4Close", "4Close")
        addOption("SubSide", "SubSide")

    }

    init {
        SmartDashboard.putData("Best Song Lyrics", lyricsChooser)
        SmartDashboard.putData("Tests", testAutoChooser)
        SmartDashboard.putData("Autos", autonomousChooser)
        closeFourToFiveEntry.setBoolean(false)
        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                val autoChosen = selAuto == "no auto selected" || selAuto != "Tests" || selAuto == ""

                SmartDashboard.putBoolean("Auto is selected", autoChosen)
            }
        }
        try {
            cacheFile = File("/home/lvuser/autonomi.json")
            if (cacheFile != null) {
                autonomi = Autonomi.fromJsonString(cacheFile?.readText())!!
                println("Autonomi cache loaded.")
            } else {
                println("Autonomi failed to load!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! RESTART ROBOT!!!!!!")
            }
        } catch (_: Throwable) {
            DriverStation.reportError("Autonomi cache could not be found", false)
            autonomi = Autonomi()
        }
        println("In Auto Init. Before AddListener. Hi.")
        NetworkTableInstance.getDefault().addListener(
            autonomiEntryTopicSub,
            EnumSet.of(
                NetworkTableEvent.Kind.kImmediate,
                NetworkTableEvent.Kind.kPublish,
                NetworkTableEvent.Kind.kValueAll
            )
        ) { event ->
            println("Autonomous change detected")
            if (event.valueData != null) {
                val json = event.valueData.value.string
                if (json.isNotEmpty()) {
                    val t = measureTimeFPGA {
                        autonomi = Autonomi.fromJsonString(json) ?: Autonomi()
                    }
                    println("Loaded autonomi in $t seconds")
                    if (cacheFile != null) {
                        println("CacheFile != null. Hi.")
                        cacheFile!!.writeText(json)
                    } else {
                        println("cacheFile == null. Hi.")
                    }
                    println("New autonomi written to cache")
                } else {
                    autonomi = Autonomi()
                    DriverStation.reportWarning("Empty autonomi received from network tables", false)
                }
            }
        }
    }

    @OptIn(DelicateCoroutinesApi::class)
    suspend fun autonomous() = use(Drive, name = "Autonomous") {
        println("Got into Auto fun autonomous. Hi. 888888888888888 ${Robot.recentTimeTaken()}")
        SmartDashboard.putString("autoStatus", "init")
        println("Selected Auto = *****************   $selAuto ****************************  ${Robot.recentTimeTaken()}")
        resetCameras()
        when (selAuto) {
            "HIII" -> hiii()
            "Tests" -> testAuto()
            "2Far2CloseAmp" -> twoFarTwoCloseAmp()
            "4Close" -> fourClose()
            "SubSide" -> substationSide()
            else -> println("No function found for ---->$selAuto<-----  ${Robot.recentTimeTaken()}")
        }
        SmartDashboard.putString("autoStatus", "complete")
        println("finished autonomous  ${Robot.recentTimeTaken()}")
    }

    suspend fun twoFarTwoCloseAmp() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition = if (isRedAlliance) Vector2(48.32, 21.78) else Vector2(3.95, 21.78) //sets position the starting position FOR RED ONLY!!! //47.6, 20.6)
            val auto = autonomi["2Far2CloseAmp"]
            auto?.isReflected = isBlueAlliance
            var path: Path2D? = auto?.get("1-Start")

//            Pivot.angleSetpoint = 48.0.degrees //Shooter.pitchCurve.getValue(Pivot.distFromSpeaker).degrees
            aimAndShoot() //preLoaded shot

            pickUpSeenNote(if (PoseEstimator.apriltagsEnabled) 0.7 else 0.3)
            suspendUntil{ Intake.intakeState == Intake.IntakeState.HOLDING }
            println("Staged!")
            aimAndShoot() //second note shot

            if (path != null) {
                if (!PoseEstimator.apriltagsEnabled) path.scaleEasePoints(4.0)
                Drive.driveAlongPath(path,  false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote(if (PoseEstimator.apriltagsEnabled) 0.6 else 0.3)
            path = auto?.get("2-ShootThird")
            if (path != null) {
                if (!PoseEstimator.apriltagsEnabled) path.scaleEasePoints(4.0)
                Drive.driveAlongPath(path, false)
            }

            delay(0.2) //Waiting for AprilTag to catch up
            aimAndShoot() //third note shot

            path = auto?.get("3-GrabFourth")
            if (path != null) {
                if (!PoseEstimator.apriltagsEnabled) path.scaleEasePoints(6.0)
                Drive.driveAlongPath(path,  false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }

            pickUpSeenNote(if (PoseEstimator.apriltagsEnabled) 0.4 else 0.3, timeOut = false)
        } finally {
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
        }
    }

    suspend fun fourClose() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition =
                if (isRedAlliance) Vector2(49.54, 13.49) else Vector2(49.54, 13.49).reflectAcrossField()
            val auto = autonomi["4Close"]
            auto?.isReflected = isBlueAlliance
            var path = auto?.get("1-GrabSecond")

            aimAndShoot()
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote(1.0)
            suspendUntil{ Intake.intakeState == Intake.IntakeState.HOLDING }
            aimAndShoot()
            path = auto?.get("2-GrabThird")
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote(1.0)
            suspendUntil{ Intake.intakeState == Intake.IntakeState.HOLDING }
            aimAndShoot()
            path = auto?.get("3-GrabFourth")
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote(1.0)
            suspendUntil{ Intake.intakeState == Intake.IntakeState.HOLDING }
            aimAndShoot()

            if (closeFourToFiveEntry.getBoolean(false)) {
                path = auto?.get("4-GrabFifth")
                if (path != null) {
                    Drive.driveAlongPath(path, false, earlyExit = {
                        NoteDetector.seesNote && NoteDetector.closestIsValid
                    })
                }
                pickUpSeenNote(0.8, cautious = true)
                path = auto?.get("5-ShootFifth")
                if (path != null) {
                    Drive.driveAlongPath(path, false)
                }
                aimAndShoot()
            }
        } finally {
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
        }
    }

    suspend fun substationSide() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition =
                if (isRedAlliance) Vector2(46.0, 11.95) else Vector2(46.0, 11.95).reflectAcrossField()
            val auto = autonomi["SubSide"]
            auto?.isReflected = isBlueAlliance
            var path = auto?.get("1-GrabSecond")
            aimAndShoot()
            if (path != null) {
                Drive.driveAlongPath(path, true, inResetGyro = false, earlyExit = {
                    /*Drive.combinedPosition.x > 23.0 &&*/ NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            println("finished path")
//            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote(0.5)
            path = auto?.get("2-ShootSecond")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            aimAndShoot()
            path = auto?.get("3-GrabThird")
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
//            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote(0.5)
            path = auto?.get("4-ShootThird")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            aimAndShoot()
            path = auto?.get("5-GrabFourth")
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
//            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote(0.5)
            path = auto?.get("6-ShootFourth")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            aimAndShoot()
        } finally {
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
        }
    }


    private suspend fun testAuto() = use(Drive) {
        val testPath = SmartDashboard.getString("Tests/selected", "no test selected") // testAutoChooser.selected
        if (testPath != null) {
            val testAutonomous = autonomi["Tests"]
            val path = testAutonomous?.get(testPath)
            println(testPath)
            if (path != null) {
                println("Path is not null")
                Drive.driveAlongPath(path, true)
            } else {
                println("Path is null!!! :(")
            }
        }
    }
    fun hiii() {
        println("hiiiiiiiiiiiiiiiiiiiiiiiiiiiiii")
    }
}


