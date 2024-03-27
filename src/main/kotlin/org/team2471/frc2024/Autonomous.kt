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
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion.following.xPose
import org.team2471.frc.lib.motion_profiling.Autonomi
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.Timer
import org.team2471.frc.lib.util.measureTimeFPGA
import org.team2471.frc2024.AprilTag.aprilTagsEnabled
import org.team2471.frc2024.AprilTag.resetCameras
import org.team2471.frc2024.Drive.combinedPosition
import org.team2471.frc2024.Drive.isBlueAlliance
import org.team2471.frc2024.Drive.isRedAlliance
import java.io.File
import java.util.*
import kotlin.math.absoluteValue

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
private val shootFirstEntry = NetworkTableInstance.getDefault().getTable("Autos").getEntry("ShootFirst")
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
//        addOption("2Far2CloseAmp", "2Far2CloseAmp")
        addOption("4CloseSafe", "4CloseSafe")
//        addOption("SubSide", "SubSide")
        addOption("SafeSubSide", "SafeSubSide")
//        addOption("testingARoundTheStageBlue", "CirclePathes")
        addOption("MidPieces", "MidPieces")
        addOption("Pile", "Pile")
        addOption("FirstMid", "FirstMid")
        addOption("FirSec", "FirSec")
        addOption("hii", "hii")
    }

    init {
        SmartDashboard.putData("Best Song Lyrics", lyricsChooser)
        SmartDashboard.putData("Tests", testAutoChooser)
        SmartDashboard.putData("Autos", autonomousChooser)
        closeFourToFiveEntry.setBoolean(true)
        shootFirstEntry.setBoolean(true)
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
//                        println(autonomi.getPath("Tests", "8 Foot Straight").getPosition(0.0))
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
        println("Got into Auto fun autonomous. Hi. 888888888888888 ${Robot.totalTimeTaken()}")
//        SmartDashboard.putString("autoStatus", "init")
        println("Selected Auto = *****************   $selAuto ****************************  ${Robot.totalTimeTaken()}")
        when (selAuto) {
            "Tests" -> testAuto()
//            "2Far2CloseAmp" -> twoFarTwoCloseAmp()
            "4CloseSafe" -> fourCloseSafe()
//            "SubSide" -> substationSide()
            "SafeSubSide" -> safeSubstationSide()
//            "testingARoundTheStageBlue" -> testingARoundTheStageBlue()
            "MidPieces" -> midPieces()
            "Pile" -> pileAuto()
            "FirstMid" -> firstMidAuto()
            "FirSec" -> firstSecAuto()
            "hii" -> hii()
            else -> println("No function found for ---->$selAuto<-----  ${Robot.totalTimeTaken()}")
        }
        SmartDashboard.putString("autoStatus", "complete")
        println("finished autonomous  ${Robot.totalTimeTaken()}")
    }

    suspend fun twoFarTwoCloseAmp() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition = if (isRedAlliance) Vector2(48.32, 21.78).feet else Vector2(48.32, 21.78).reflectAcrossField().feet //sets position the starting position FOR RED ONLY!!! //47.6, 20.6)
            val auto = autonomi["2Far2CloseAmp"]
            auto?.isReflected = isBlueAlliance
            var path: Path2D? = auto?.get("0.5-GrabSecond")
            val ti = Timer()
            if (shootFirstEntry.getBoolean(true)) {
                println("shooter ramp in Preload ")
                aimAndShoot() //preLoaded shot
            }
            println(" shooter after preload")
            Intake.intakeState = Intake.IntakeState.INTAKING
            ti.start()
            if (path != null) {
                Drive.driveAlongPath(path,  false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            pickUpSeenNote()
            suspendUntil{ Intake.intakeState == Intake.IntakeState.HOLDING || ti.get() > 0.4}
            aimAndShoot() //second note shot

            Intake.intakeState = Intake.IntakeState.INTAKING
            if (path != null) {
                Drive.driveAlongPath(path,  false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }
            if (!NoteDetector.seesNote) delay(0.2)
            pickUpSeenNote()
            path = auto?.get("2-ShootThird")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }

            delay(0.3) //Waiting for AprilTag to catch up
            aimAndShoot() //third note shot

            Intake.intakeState = Intake.IntakeState.INTAKING
            path = auto?.get("3-GrabFourth")
            if (path != null) {
                Drive.driveAlongPath(path,  false, earlyExit = {
                    NoteDetector.seesNote && NoteDetector.closestIsValid
                })
            }

            pickUpSeenNote(timeOut = false)

            if (closeFourToFiveEntry.getBoolean(false)) {
                path = auto?.get("4-ShootFourth")
                if (path != null) {
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
            println("DONE GRABBING FIRST FOUR. GOING TO MIDDLE")

            Shooter.setRpms(0.0)
            val auto = autonomi["MidPieces"]
            auto?.isReflected = isRedAlliance
            var path: Path2D? = auto?.get("4-GrabFifth")


            if (path != null) {
                Intake.intakeState = Intake.IntakeState.INTAKING
                Drive.driveAlongPath(path, earlyExit = {
                    it > 0.5 && NoteDetector.closestIsValid
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
                    it > 0.3 && NoteDetector.closestIsValid
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

            Drive.xPose()

        } finally {
            Drive.xPose()
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
            delay(1.0)
            Shooter.setRpms(0.0)
            Intake.setIntakeMotorsPercent(0.0)
        }
    }

    private suspend fun grabFirstFour() = use(Drive, Shooter, Intake) {
        Shooter.setRpms(5000.0)

        Drive.zeroGyro()
        val auto = autonomi["MidPieces"]
        auto?.isReflected = isRedAlliance
        var path = auto?.get("1-GrabSecond")

        if (path != null) combinedPosition = path.getPosition(0.0).feet

        aimAndShoot(true)

        var finishedPath = false

        parallel({
            if (path != null) {
                parallel({
                    Drive.driveAlongPath(path!!, false, turnOverride = { Drive.aimSpeakerAmpLogic() }/*, earlyExit = {
                NoteDetector.closestIsValid && !NoteDetector.closestIsMiddle && it > 0.5
            }*/)
                }, {
                    delay(0.6)
                    Pivot.aimSpeaker = false
                    Shooter.setRpms(5000.0)
                    Pivot.angleSetpoint = Pivot.getAngleFromPosition(path!!.getPosition(path!!.durationWithSpeed)) - 4.0.degrees
                    println("setting the pivot angle setpoint. path duration: ${path!!.durationWithSpeed}  angle setpoint: ${Pivot.angleSetpoint}")
                })
            }
//            pickUpSeenNote(true, doTurn = false)
            parallel({
                path = auto?.get("2-GrabThird")
                if (path != null) {
                    Drive.driveAlongPath(path!!, false, turnOverride = { Drive.aimSpeakerAmpLogic() }/*, earlyExit = {
                NoteDetector.closestIsValid && !NoteDetector.closestIsMiddle && it > 0.5
            }*/)
                }
            }, {
                delay(0.8)
                Pivot.aimSpeaker = true
                Pivot.aimSpeakerDistanceOffset = 2.0
            })

//            pickUpSeenNote(true, doTurn = false)
            path = auto?.get("3-GrabFourth")
            if (path != null) {
                Pivot.aimSpeakerDistanceOffset = 2.0
                Drive.driveAlongPath(path!!, false, turnOverride = { Drive.aimSpeakerAmpLogic() }/*, earlyExit = {
                NoteDetector.closestIsValid && !NoteDetector.closestIsMiddle && it > 0.5
            }*/)
//                pickUpSeenNote(true, doTurn = false)
            }
            finishedPath = true
        }, {
            Pivot.aimSpeaker = true
//            delay(0.4)
//            println("firing preloaded")
            Intake.setIntakeMotorsPercent(1.0)
//            delay(0.6)
//            aprilTagsEnabled = false

//            periodic {
//                if (finishedPath) this.stop()
//                Drive.headingSetpoint = Drive.getAngleToSpeaker()
//            }
        })
//        aprilTagsEnabled = true
        Pivot.aimSpeakerDistanceOffset = 0.0
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
            Drive.xPose()
        } finally {
            Drive.xPose()
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
                if (isRedAlliance) Vector2(48.62, 11.62).feet else Vector2(48.52, 11.62).reflectAcrossField().feet
            val auto = autonomi["SafeSubSide"]
            auto?.isReflected = isBlueAlliance
            var path = auto?.get("1-GrabSecond")
            Shooter.setRpms(5000.0)
            var t = Timer()
            t.start()
            suspendUntil { (Drive.heading - if (isRedAlliance) 180.0.degrees else 0.0.degrees).asDegrees.absoluteValue < 10.0 || t.get() > 0.7 }
            if (shootFirstEntry.getBoolean(true)) {
                Pivot.angleFudge = 2.0.degrees
                aimAndShoot()
                Intake.intakeState = Intake.IntakeState.INTAKING
                Shooter.setRpms(0.0)
                Pivot.angleFudge = 0.0.degrees
            }
            parallel({
                if (path != null) {
                    Drive.driveAlongPath(path!!, false, inResetGyro = false, earlyExit = {
                        it > 0.5 && NoteDetector.closestNoteAtPosition(NoteDetector.middleNote(0), 5.0)
                    })
                }
            }, {
                if (!shootFirstEntry.getBoolean(true)) {
                    fire(0.5)
                    Intake.intakeState = Intake.IntakeState.INTAKING
                    Shooter.setRpms(0.0)
                }
            })
            pickUpSeenNote()

            path = auto?.get("2-ShootSecond") //Mid1 Third
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            aimAndShoot()
            Intake.intakeState = Intake.IntakeState.INTAKING
            Shooter.setRpms(0.0)
            path = auto?.get("3-GrabThird") //Mid1 Fourth
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.closestIsMiddleAdjust(4.5)
                })
            }
            pickUpSeenNote()
            path = auto?.get("4-ShootThird") //Mid1 Fourth
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
                    NoteDetector.closestIsMiddle
                })
            }
            pickUpSeenNote()
            path = auto?.get("6-ShootFourth")
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            aimAndShoot()
            Drive.xPose()
        } finally {
            Drive.xPose()
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
        }
    }

    suspend fun pileAuto() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition = //edit to actual startPos
                if (isRedAlliance) Vector2(48.52, 4.62).feet else Vector2(48.52, 4.62).reflectAcrossField().feet
            val auto = autonomi["Pile"]
            auto?.isReflected = isBlueAlliance
//            Intake.intakeState = Intake.IntakeState.INTAKING
//            var path: Path2D? = auto?.get("1-SpitFirst")
//            var t = Timer()
//            t.start()
//            parallel({
//                if (path != null) Drive.driveAlongPath(path as Path2D, true)
//            }, {
//                suspendUntil { Intake.intakeState == Intake.IntakeState.HOLDING || t.get() + 0.1 > (path?.duration ?: 10.0) }
//                Shooter.setRpms(1000.0)
//                suspendUntil { Shooter.motorRpmTop > 800.0 || t.get() + 0.1 > (path?.duration ?: 10.0)}
//                fire()
//            })

            var path = auto?.get("1-KnockNotes")
            if (path != null) Drive.driveAlongPath(path, false)
            path = auto?.get("3-GrabSecond")
            if (path != null) Drive.driveAlongPath(path, false, extraTime = 0.5)
            pickUpSeenNote()

            path = auto?.get("4-ShootSecond")
            if (path != null) Drive.driveAlongPath(path, false)
            aimAndShoot()

            pickUpSeenNote()
        } finally {
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
        }
    }

    suspend fun firstMidAuto() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition =
                if (isRedAlliance) Vector2(48.62, 11.62).feet else Vector2(48.52, 11.62).reflectAcrossField().feet
            val auto = autonomi["SafeSubSide"]
            auto?.isReflected = isBlueAlliance
            var path = auto?.get("0-Mid1_Grab1.5")
            Shooter.setRpms(5000.0)
            var t = Timer()
            t.start()
            suspendUntil { (Drive.heading - if (isRedAlliance) 180.0.degrees else 0.0.degrees).asDegrees.absoluteValue < 10.0 || t.get() > 0.7 }
            if (shootFirstEntry.getBoolean(true)) {
                Pivot.angleFudge = 2.0.degrees
                aimAndShoot()
                Intake.intakeState = Intake.IntakeState.INTAKING
                Shooter.setRpms(0.0)
                Pivot.angleFudge = 0.0.degrees
            }
            parallel({
                if (path != null) {
                    Drive.driveAlongPath(path!!, false, inResetGyro = false, earlyExit = {
                        it > 0.5 && NoteDetector.closestNoteAtPosition(NoteDetector.middleNote(2), 5.0)
                    })
                }
            }, {
                if (!shootFirstEntry.getBoolean(true)) {
                    fire(0.5)
                    Intake.intakeState = Intake.IntakeState.INTAKING
                    Shooter.setRpms(0.0)
                }
            })
            pickUpSeenNote()
            path = auto?.get("6-ShootFourth") //Mid1 Second
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            aimAndShoot()

            Intake.intakeState = Intake.IntakeState.INTAKING
            Shooter.setRpms(0.0)
            path = auto?.get("0.1-Mid1_GrabSecond") //Mid1 Third
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.closestNoteAtPosition(NoteDetector.middleNote( 0))
                })
            }
            pickUpSeenNote()

            path = auto?.get("2-ShootSecond") //Mid1 Third
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            aimAndShoot()
            Intake.intakeState = Intake.IntakeState.INTAKING
            Shooter.setRpms(0.0)
            path = auto?.get("3-GrabThird") //Mid1 Fourth
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    NoteDetector.closestIsMiddleAdjust(4.5)
                })
            }
            pickUpSeenNote()
            path = auto?.get("4-ShootThird") //Mid1 Fourth
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            aimAndShoot()
            Drive.xPose()

        } finally {
            Drive.xPose()
            Drive.aimSpeaker = false
            Pivot.aimSpeaker = false
        }
    }

    suspend fun firstSecAuto() = use(Drive, Shooter) {
        try {
            Drive.zeroGyro()
            Drive.combinedPosition =
                if (isRedAlliance) Vector2(48.62, 11.62).feet else Vector2(48.52, 11.62).reflectAcrossField().feet
            val auto = autonomi["SafeSubSide"]
            auto?.isReflected = isBlueAlliance
            var path = auto?.get("0.3-Fir2_Grab1.5")
            Shooter.setRpms(5000.0)
            var t = Timer()
            t.start()
            suspendUntil { (Drive.heading - if (isRedAlliance) 180.0.degrees else 0.0.degrees).asDegrees.absoluteValue < 10.0 || t.get() > 0.5 }
            if (shootFirstEntry.getBoolean(true)) {
                Pivot.angleFudge = 2.0.degrees
                aimAndShoot()
                Intake.intakeState = Intake.IntakeState.INTAKING
                Shooter.setRpms(0.0)
                Pivot.angleFudge = 0.4.degrees
            }
            parallel({
                if (path != null) {
                    Drive.driveAlongPath(path!!, false, inResetGyro = false, earlyExit = {
                        it > 0.5 && NoteDetector.closestNoteAtPosition(NoteDetector.middleNote(1), 7.0)
                    })
                }
            }, {
                if (!shootFirstEntry.getBoolean(true)) {
                    fire(0.5)
                    Intake.intakeState = Intake.IntakeState.INTAKING
                    Shooter.setRpms(0.0)
                    Pivot.angleFudge = 0.0.degrees
                }
            })
            pickUpSeenNote()

            path = auto?.get("4-ShootThird") //FirSe Second
            parallel({
                if (path != null) {
                    Drive.driveAlongPath(path as Path2D, false)
                }
            }, {
                delay(2.0)
            })
            aimAndShoot()

            Intake.intakeState = Intake.IntakeState.INTAKING
            Shooter.setRpms(0.0)
            path = auto?.get("5-GrabFourth") //FirSe Third
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    it > 0.5 && NoteDetector.closestNoteAtPosition(NoteDetector.middleNote(2))
                })
            }
            pickUpSeenNote()

            path = auto?.get("6-ShootFourth") //FirSe Third
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            delay(0.05)
            aimAndShoot()

            Intake.intakeState = Intake.IntakeState.INTAKING
            Shooter.setRpms(0.0)
            path = auto?.get("0.1-Mid1_GrabSecond") //FirSe Fourth
            if (path != null) {
                Drive.driveAlongPath(path, false, earlyExit = {
                    it > 0.5 && NoteDetector.closestNoteAtPosition(NoteDetector.middleNote(0))
                })
            }
            pickUpSeenNote()

            path = auto?.get("2-ShootSecond") //FirSe Fourth
            if (path != null) {
                Drive.driveAlongPath(path, false)
            }
            Shooter.setRpms(5000.0)
            aimAndShoot()
            Drive.xPose()
        } finally {
            Drive.xPose()
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

    suspend fun hii() = use(Drive, Shooter, Intake, name = "hii") {
        println("hiiiii ${Robot.totalTimeTaken()}")
        Drive.zeroGyro()
        Drive.combinedPosition =
            if (isRedAlliance) Vector2(48.62, 11.62).feet else Vector2(48.52, 11.62).reflectAcrossField().feet
        val auto = autonomi["SafeSubSide"]
        auto?.isReflected = isBlueAlliance
        var path = auto?.get("0-Mid1_Grab1.5")

        val t = Timer()
        t.start()

        val startingPosition = Drive.combinedPosition

        println("line in front of driveAlongPath ${Robot.totalTimeTaken()}")
        if (path != null) Drive.driveAlongPath(path, earlyExit = { true })
        println("line after driveAlongPath ${Robot.totalTimeTaken()}")



    }
//
//    suspend fun bye() {
//        println("bye ${Robot.totalTimeTaken()}")
//        Drive.zeroGyro()
//        Drive.combinedPosition =
//            if (isRedAlliance) Vector2(48.62, 11.62).feet else Vector2(48.52, 11.62).reflectAcrossField().feet
//        val auto = autonomi["4CloseSafe"]
//        auto?.isReflected = isRedAlliance
//        var path = auto?.get("1-GrabSecond")
////        println(path?.getPosition(0.0))
//
//        val t = Timer()
//        t.start()
//
//        val startingPosition = Drive.combinedPosition
//
//        println("line in front of driveAlongPath ${Robot.totalTimeTaken()}")
//        if (path != null) Drive.driveAlongPath(path, earlyExit = { true })
//        println("line after driveAlongPath ${Robot.totalTimeTaken()}")
//    }
}


