package org.team2471.bunnybots2023

import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
//import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.periodic
//import org.team2471.bunnybots2022.Drive
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.drive
import org.team2471.frc.lib.motion.following.driveAlongPath
import org.team2471.frc.lib.motion_profiling.Autonomi
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.util.Timer
import org.team2471.frc.lib.util.measureTimeFPGA
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

private var startingSide = Side.RIGHT
val selAuto
    get() = SmartDashboard.getString("Autos/selected", "no auto selected")

object AutoChooser {
    private val isRedAllianceEntry = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("isRedAlliance")
    private var autonomiEntryTopicSub =
        NetworkTableInstance.getDefault().getTable("PathVisualizer").getStringTopic("Autonomi").subscribe("")

    var cacheFile: File? = null
    var redSide: Boolean = true
        get() = isRedAllianceEntry.getBoolean(true)
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
        addOption("Hook Path", "Hook Path")
        setDefaultOption("90 Degree Turn", "90 Degree Turn")


    }

    private val autonomousChooser = SendableChooser<String?>().apply {
        setDefaultOption("Tests", "testAuto")
        addOption("Outer Three Auto", "outerThreeAuto")
        addOption("Outer Two Auto", "outerTwoAuto")
        addOption("Inner Three Auto", "innerThreeAuto")
        addOption("NodeDeck", "nodeDeck")
        addOption("BunnyBot2023", "BunnyBot2023")

    }

    init {
//        DriverStation.reportWarning("Starting auto init warning", false)
//        DriverStation.reportError("Starting auto init error", false)         //            trying to get individual message in event log to get timestamp -- untested

        SmartDashboard.putData("Best Song Lyrics", lyricsChooser)
        SmartDashboard.putData("Tests", testAutoChooser)
        SmartDashboard.putData("Autos", autonomousChooser)

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
    private suspend fun bunnyBot2023() = use(Drive, Shooter) {
        println("inside bunnyBot2023() auto function")
        Drive.initializeSteeringMotors()
        Drive.zeroGyro()
        Limelight.ledModeEntry.setDouble(0.0)
        Intake.intakeUp()
        val totePath = autonomi["BunnyBot2023"]?.get("MoveToTotes")
        var doneWithPath = false
        if (totePath != null) {
            parallel({
                println("GONNA DRIVE NOWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")
                Drive.driveAlongPath(totePath, true)
                doneWithPath = true
            }, {
                val t = Timer()
                var waitTime = 0.0
                t.start()
                periodic {
                    if (doneWithPath) {
                        Drive.abortPath()
                        this.stop()
                    } else {
                        if (Limelight.seesTargets /*&& t.get() - waitTime > 1.0*/ || !Robot.isAutonomous) {
                            println("saw target for more then 1 second, aborting path")
                            doneWithPath = true
                        } else {
                            waitTime = t.get()
                        }
                    }

                }
            })
            val t = Timer()
            val tTwo = Timer()
            var blindTime = 0.0
            var waiting = false
            var shooting = false
            var looking = true
            tTwo.start()
            periodic {
                if (Robot.isAutonomous) {
                    if (Limelight.seesTargets) {

//                        println("driving to bucket at robot centric:${Limelight.enemyBuckets[0].botCentCoords}  field centric: ${Limelight.enemyBuckets[0].botCentCoords + Drive.position} from ${Drive.position} ")

                        if (Limelight.enemyBuckets[0].dist > 4.0.feet) {
                            Drive.drive(
                                Vector2(
                                    Limelight.enemyBuckets[0].botCentCoords.x  /*Limelight.botCentFilterX.calculate(Limelight.enemyBuckets[0].botCentCoords.x)*/,
                                    Limelight.enemyBuckets[0].botCentCoords.y/*Limelight.botCentFilterY.calculate(Limelight.enemyBuckets[0].botCentCoords.y)*/
                                ).normalize() * 0.5,
                                0.0,
                                false
                            )
                        }
                        if (Limelight.enemyBuckets.isNotEmpty()) {
                            println("turret Error ${Turret.turretError.asDegrees.absoluteValue} distance ${Limelight.enemyBuckets[0].dist} ")
                        }

                        if (Limelight.enemyBuckets.isNotEmpty()) {
                            if (waiting) {
                                Shooter.uptakeMotor.setPercentOutput(0.0)
                                if (t.get() > 1.2) {
                                    waiting = false
                                    looking = true
                                }
                                println("waiting")
                                if (!Shooter.ballReady) {
                                    Shooter.uptakeMotor.setPercentOutput(1.0)
                                } else {
                                    Shooter.uptakeMotor.setPercentOutput(0.0)
                                }

                            } else if (shooting) {
                                Shooter.uptakeMotor.setPercentOutput(1.0)
                                if (!Shooter.ballReady) {
                                    shooting = false
                                    waiting = true
                                    t.start()
                                }

                                println("shooting")
                            } else if (looking) {
                                if (Turret.turretError.asDegrees.absoluteValue < 5.0 && Limelight.enemyBuckets[0].dist < 5.0.feet && Shooter.ballReady) {
                                    looking = false
                                    shooting = true
                                    t.start()
                                }
                                if (!Shooter.ballReady) {
                                    Shooter.uptakeMotor.setPercentOutput(1.0)
                                } else {
                                    Shooter.uptakeMotor.setPercentOutput(0.0)
                                }
                                if (Limelight.enemyBuckets[0].dist < 5.0.feet) {
                                    Drive.drive(
                                        Vector2(0.0, 0.0),
                                        0.0,
                                        false
                                    )
                                } else {
                                    Drive.drive(
                                        Vector2(
                                            Limelight.enemyBuckets[0].botCentCoords.x  /*Limelight.botCentFilterX.calculate(Limelight.enemyBuckets[0].botCentCoords.x)*/,
                                            Limelight.enemyBuckets[0].botCentCoords.y/*Limelight.botCentFilterY.calculate(Limelight.enemyBuckets[0].botCentCoords.y)*/
                                        ).normalize() * 0.5,
                                        0.0,
                                        false
                                    )
                                }
                                Turret.rawTurretSetpoint += 1.0.degrees
                                println("looking")
                            }
                            if (looking && !shooting && !waiting && tTwo.get() - blindTime > 2.0) {
                                println("GONNA DRIVE NOWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")
                                Drive.drive(
                                    Vector2(0.0, 1.0),
                                    0.0, false
                                )
                            } else {
                                blindTime = tTwo.get()
                                Drive.drive(
                                    Vector2(0.0, 0.0),
                                    0.0,
                                    false
                                )
                            }
                        }
                        //   turret shoot at target
                    } else {
                        Drive.drive(
                            Vector2(0.0, 0.0),
                            0.0,
                            false
                        )
                        Turret.rawTurretSetpoint += 1.0.degrees

                        Shooter.uptakeMotor.setPercentOutput(0.0)
                        println("i no see!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
//                        waitTime = t.get()
                    }
                } else {
                    this.stop()
                }
            }
            //drive to target and shoot
//                }

        } else {
            println("BUNNYBOTS PATH IS NULL!!!!!!!!")
        }

        // drive, shoot, intake, intake motors, limeLight
    }

    suspend fun autonomous() = use(Drive, name = "Autonomous") {
        println("Got into Auto fun autonomous. Hi. 888888888888888 ${Robot.recentTimeTaken()}")
        SmartDashboard.putString("autoStatus", "init")
        println("Selected Auto = *****************   $selAuto ****************************  ${Robot.recentTimeTaken()}")
        when (selAuto) {
            "Tests" -> testAuto()
            "BunnyBot2023" -> bunnyBot2023()
            else -> println("No function found for ---->$selAuto<-----  ${Robot.recentTimeTaken()}")
        }
        SmartDashboard.putString("autoStatus", "complete")
        println("finished autonomous  ${Robot.recentTimeTaken()}")
    }


    private suspend fun testAuto() {
        val testPath = SmartDashboard.getString("Tests/selected", "no test selected") // testAutoChooser.selected
        if (testPath != null) {
            val testAutonomous = autonomi["Tests"]
            val path = testAutonomous?.get(testPath)
            if (path != null) {
                Drive.driveAlongPath(path, true)
            }
        }
    }




}


