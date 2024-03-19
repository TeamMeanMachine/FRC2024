package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.feet
import org.team2471.frc.lib.math.inches
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion_profiling.Autonomi
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer
import org.team2471.frc.lib.util.measureTimeFPGA
import org.team2471.frc2024.AprilTag.resetCameras
import org.team2471.frc2024.Drive.isBlueAlliance
import org.team2471.frc2024.Drive.isRedAlliance
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
    private val closeFourToFiveEntry = AprilTag.aprilTable.getEntry("Auto, Go For It")
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
        addOption("4CloseSafe", "4CloseSafe")
        addOption("SubSide", "SubSide")
        addOption("SafeSubSide", "SafeSubSide")
        addOption("testingARoundTheStageBlue", "CirclePathes")
        addOption("MidPieces", "MidPieces")
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
        println("reset cams ${Robot.recentTimeTaken()}")
        when (selAuto) {
            "HIII" -> hiii()
            "Tests" -> testAuto()
            "2Far2CloseAmp" -> twoFarTwoCloseAmp()
            "4CloseSafe" -> fourCloseSafe()
            "SubSide" -> substationSide()
            "SafeSubSide" -> safeSubstationSide()
            "testingARoundTheStageBlue" -> testingARoundTheStageBlue()
            "MidPieces" -> midPieces()
            else -> println("No function found for ---->$selAuto<-----  ${Robot.recentTimeTaken()}")
        }
        SmartDashboard.putString("autoStatus", "complete")
        println("finished autonomous  ${Robot.recentTimeTaken()}")
    }

    suspend fun twoFarTwoCloseAmp() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition = if (isRedAlliance) Vector2(48.32, 21.78).feet else Vector2(48.32, 21.78).reflectAcrossField().feet //sets position the starting position FOR RED ONLY!!! //47.6, 20.6)
            val auto = autonomi["2Far2CloseAmp"]
            auto?.isReflected = isBlueAlliance
            var path: Path2D? = auto?.get("0.5-GrabSecond")
            val ti = Timer()

            aimAndShoot() //preLoaded shot

            Intake.intakeState = Intake.IntakeState.INTAKING
            ti.start()
            if (path != null) {
                if (!AprilTag.aprilTagsEnabled) path.scaleEasePoints(0.2)
                Drive.driveAlongPath(path,  false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote()
            suspendUntil{ Intake.intakeState == Intake.IntakeState.HOLDING || ti.get() > 0.4}
            aimAndShoot() //second note shot

            Intake.intakeState = Intake.IntakeState.INTAKING
            if (path != null) {
                if (!AprilTag.aprilTagsEnabled) path.scaleEasePoints(3.5)
                Drive.driveAlongPath(path,  false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote()
            path = auto?.get("2-ShootThird")
            if (path != null) {
                if (!AprilTag.aprilTagsEnabled) path.scaleEasePoints(3.0)
                Drive.driveAlongPath(path, false)
            }

            delay(0.3) //Waiting for AprilTag to catch up
            aimAndShoot() //third note shot

            Intake.intakeState = Intake.IntakeState.INTAKING
            path = auto?.get("3-GrabFourth")
            if (path != null) {
                if (!AprilTag.aprilTagsEnabled) path.scaleEasePoints(3.5)
                Drive.driveAlongPath(path,  false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }

            pickUpSeenNote(timeOut = false)

            if (closeFourToFiveEntry.getBoolean(false)) {
                path = auto?.get("4-ShootFourth")
                if (path != null) {
                    if (!AprilTag.aprilTagsEnabled) path.scaleEasePoints(4.0)
                    Drive.driveAlongPath(path, false)
                }

                delay(0.3) //Waiting for AprilTag to catch up
                aimAndShoot() //third note shot
            }
        } finally {
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
        }
    }

    suspend fun midPieces() = use(Drive, Shooter) {
        try {
            grabFirstFour() //split because we want intake default for picking up middle pieces

            Shooter.setRpms(0.0)
            val auto = autonomi["MidPieces"]
            auto?.isReflected = isRedAlliance
            var path: Path2D? = auto?.get("4-GrabFifth")


            if (path != null) {
                Intake.intakeState = Intake.IntakeState.INTAKING
                Drive.driveAlongPath(path, earlyExit = {
                    it > 0.25 && NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote()
            path = auto?.get("5-ShootFifth")
            parallel({
                delay(0.5)
                Intake.setIntakeMotorsPercent(0.0)
                Shooter.setRpms(5000.0)
                Pivot.aimSpeaker = true
            }, {
                if (path != null) {
                    Drive.driveAlongPath(path!!, turnOverride = {Drive.aimSpeakerAmpLogic()})
                }
            })
            if (Intake.holdingCargo) {
                println("intake is holding, shooting")
                aimAndShoot(true)
            } else {
                println("Intake is not holding, not shooting")
            }
            Shooter.setRpms(0.0)
            path = auto?.get("6-GrabSixth")
            if (path != null) {
                Intake.intakeState = Intake.IntakeState.INTAKING
                Drive.driveAlongPath(path, earlyExit = {
                    it > 0.25 && NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote()
            path = auto?.get("7-ShootSixth")
            parallel({
                delay(0.5)
                Intake.setIntakeMotorsPercent(0.0)
                Shooter.setRpms(5000.0)
                Pivot.aimSpeaker = true
            }, {
                if (path != null) {
                    Drive.driveAlongPath(path, turnOverride = {Drive.aimSpeakerAmpLogic()})
                }
            })
            if (Intake.holdingCargo) {
                aimAndShoot(true)
            }



        } finally {
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
            delay(1.0)
            Shooter.setRpms(0.0)
            Intake.setIntakeMotorsPercent(0.0)
        }
    }

    private suspend fun grabFirstFour() = use(Drive, Shooter, Intake) {
        Shooter.setRpms(5000.0)
        Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE
//        Pivot.aimSpeaker = true

        Drive.zeroGyro()
        val auto = autonomi["MidPieces"]
        auto?.isReflected = isRedAlliance
        var path = auto?.get("1-GrabSecond")

        if (path != null) {
            parallel({
                delay(0.1)
                println("firing preloaded")
                fire(0.5)
                Pivot.aimSpeaker = true
                Intake.setIntakeMotorsPercent(1.0)
                path = auto?.get("2-GrabThird")
            }, {
                Drive.driveAlongPath(path!!, true)
            })
        }
        if (path != null) {
            Drive.driveAlongPath(path!!, false)
        }
        Intake.intakeState = Intake.IntakeState.INTAKING
        path = auto?.get("3-GrabFourth")
        if (path != null) {
            Drive.driveAlongPath(path!!, false/*, earlyExit = {
                NoteDetector.closestIsValid
            }*/)
        }
//        pickUpSeenNote()
//        aimAndShoot()
        println("finished first three")
    }

    suspend fun fourCloseSafe() = use(Drive, Shooter, Intake) {
        try {
            println("in 4closeSafe ${Robot.recentTimeTaken()}")
            Shooter.setRpms(5000.0)
            Pivot.angleSetpoint = Pivot.CLOSESPEAKERPOSE
            Drive.zeroGyro()

            val auto = autonomi["4CloseSafe"]
            auto?.isReflected = isRedAlliance
            var path = auto?.get("1-GrabSecond")
            var finishedPaths = false

            delay(0.1)
            fire(0.6) //fire preloaded
            Intake.setIntakeMotorsPercent(1.0) //run intake throughout auto
            delay(1.0)
            parallel({ //drive do driving
                if (path != null) {
                    Pivot.angleSetpoint = if (isRedAlliance) 40.0.degrees else 40.0.degrees
                    Drive.driveAlongPath(path!!, true, useCombinedPosition = false)
                } else {
                    println("PATH EQUALS NULL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                }
                path = auto?.get("2-GrabThird")
                if (path != null) {
                    Pivot.angleSetpoint = if (isRedAlliance) 37.0.degrees else 40.0.degrees
                    Drive.driveAlongPath(path!!, false, useCombinedPosition = false)
                }
                path = auto?.get("3-GrabFourth")
                if (path != null) {
                    Pivot.angleSetpoint = if (isRedAlliance) 34.0.degrees else 36.0.degrees
                    Drive.driveAlongPath(path!!, false, useCombinedPosition = false)
                }
                delay(1.0)
                finishedPaths = true
            }, { //set pivot angle and shooter rpm based on drive pos
                delay(0.2)
                periodic {
                    if (finishedPaths) this.stop()
//                    Pivot.angleSetpoint = Shooter.pitchCurve.getValue(Drive.distanceFromSpeakerDrivePos).degrees
//                    Shooter.setRpms(Shooter.rpmCurve.getValue(Drive.distanceFromSpeakerDrivePos))
                }
            })
        } finally {
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
            Shooter.setRpms(0.0)
            Intake.setIntakeMotorsPercent(0.0)
        }
    }

    suspend fun substationSide() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition =
                if (isRedAlliance) Vector2(46.0, 11.95).feet else Vector2(46.0, 11.95).reflectAcrossField().feet
            val auto = autonomi["SubSide"]
            auto?.isReflected = isBlueAlliance
            var path = auto?.get("1-GrabSecond")
            aimAndShoot()
            Intake.intakeState = Intake.IntakeState.INTAKING
            if (path != null) {
                Drive.driveAlongPath(path, true, inResetGyro = false, earlyExit = {
                    /*Drive.combinedPosition.x > 23.0 &&*/ NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            println("finished path")
            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote()
            path = auto?.get("2-ShootSecond")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            aimAndShoot()
            Intake.intakeState = Intake.IntakeState.INTAKING
            path = auto?.get("3-GrabThird")
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote()
            path = auto?.get("4-ShootThird")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            aimAndShoot()
            Intake.intakeState = Intake.IntakeState.INTAKING
            path = auto?.get("5-GrabFourth")
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote()
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

    suspend fun safeSubstationSide() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition =
                if (isRedAlliance) Vector2(48.52, 11.62).feet else Vector2(48.52, 11.62).reflectAcrossField().feet
            val auto = autonomi["SafeSubSide"]
            auto?.isReflected = isBlueAlliance
            var path = auto?.get("1-GrabSecond")
            Shooter.setRpms(5000.0)
            aimAndShoot()
            Intake.intakeState = Intake.IntakeState.INTAKING
            Shooter.setRpms(0.0)
            if (path != null) {
                Drive.driveAlongPath(path, false, inResetGyro = false, earlyExit = {
                    NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote()
            path = auto?.get("2-ShootSecond")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            aimAndShoot()
            Intake.intakeState = Intake.IntakeState.INTAKING
            Shooter.setRpms(0.0)
            path = auto?.get("3-GrabThird")
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote()
            path = auto?.get("4-ShootThird")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            aimAndShoot()
            Intake.intakeState = Intake.IntakeState.INTAKING
            Shooter.setRpms(0.0)
            path = auto?.get("5-GrabFourth")
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote()
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

    suspend fun propAuto() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition =
                if (isRedAlliance) Vector2(0.0, 0.0).feet else Vector2(0.0, 0.0).reflectAcrossField().feet
            val auto = autonomi["Tests"]
            auto?.isReflected = isBlueAlliance
            var path = auto?.get("8 Foot Straight")
            aimAndShoot()

            if (path != null) {
                Drive.driveAlongPath(path, true, inResetGyro = false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            if (!NoteDetector.seesNote) delay(0.2)
        } finally {

        }
    }
    private suspend fun testingARoundTheStageBlue() = use(Drive) {
        val testAutonomous = autonomi["testingARoundTheStageBlue"]
        val path = testAutonomous?.get("CirclePathes")
        if (path != null) {
            println("Path is not null")
            Drive.driveAlongPath(path, true)
        } else {
            println("Path is null!!! :(")
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


