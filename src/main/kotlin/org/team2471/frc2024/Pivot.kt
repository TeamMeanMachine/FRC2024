package org.team2471.frc2024

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DriverStation
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.feet
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.Drive.combinedPosition
import org.team2471.frc2024.Drive.speakerPos
import org.team2471.frc2024.Robot.isCompBot
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.cos


object Pivot: Subsystem("Pivot") {
    private val table = NetworkTableInstance.getDefault().getTable("Pivot")

    private val pivotCurrentEntry = table.getEntry("Pivot Current")

    private val ticksEntry = table.getEntry("Pivot Ticks")
    private val encoderAngleEntry = table.getEntry("Pivot Encoder Angle")
    private val motorAngleEntry = table.getEntry("Pivot Motor Angle")
    private val angleSetpointEntry = table.getEntry("Pivot Angle Setpoint")
//    private val encoderVoltageEntry = table.getEntry("Encoder Voltage")
    private val stageAngleEntry = table.getEntry("Stage Angle")
    private val distanceFromSpeakerEntry = table.getEntry("Distance From Speaker")
    val pivotAmpRate = table.getEntry("Pivot amp rate")
//    var advantagePivotPublisher: StructPublisher<Transform3d> = NetworkTableInstance.getDefault().getStructTopic("Advantage Pivot Transform", Transform3d.struct).publish()


    val pivotMotor = MotorController(FalconID(Falcons.PIVOT))

    private val pivotEncoder = AnalogInput(AnalogSensors.PIVOT)

    private const val GEARRATIO = 1 / 61.71

    val TESTPOSE = 30.5.degrees //18 //32
    val PODIUMPOSE = 37.0.degrees
    val FARSTAGELEG = 29.0.degrees
    val CLOSESPEAKERPOSE = 59.0.degrees
    val MINHARDSTOP = 5.5.degrees
    val DRIVEPOSE = MINHARDSTOP + 2.0.degrees
    val MAXHARDSTOP = 110.2.degrees
    val AMPPOSE = 107.5.degrees

    // Ticks
    private val MINTICKS = if (isCompBot) 3551.0 else 2325.0
    private val MAXTICKS = if (isCompBot) 2374.0 else 1139.0

    var angleFudge = 0.0.degrees

//    var advantagePivotTransform = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d((Math.PI / 2) + MINHARDSTOP.asRadians, 0.0, (Math.PI / 2)))


    var aimSpeaker = false
        set(value) {
            field = value
            if (value) {
                if (OI.driverController.x && !DriverStation.isAutonomous()) {
                    Shooter.rpmTopSetpoint = Shooter.rpmCurve.getValue(10.9)
                    Shooter.rpmBottomSetpoint = Shooter.rpmTopSetpoint
                } else if (!Robot.isAutonomous) {
                    Shooter.rpmTopSetpoint = Shooter.rpmCurve.getValue(distFromSpeaker)
                    Shooter.rpmBottomSetpoint = Shooter.rpmTopSetpoint
                }
            } else {
                Shooter.rpmTopSetpoint = 0.0
                Shooter.rpmBottomSetpoint = 0.0
            }
        }

    var aimSpeakerDistanceOffset: Double = 0.0 //feet

    val pivotTicks: Int
        get() = pivotEncoder.value

    val encoderVoltage: Double
        get() = pivotEncoder.voltage

    val pivotEncoderAngle: Angle
        get() = linearMap(MINTICKS, MAXTICKS, MINHARDSTOP.asDegrees, MAXHARDSTOP.asDegrees, pivotEncoder.value.toDouble()).degrees

    val pivotMotorAngle: Angle
        get() = pivotMotor.position.degrees

    var angleSetpoint: Angle = pivotEncoderAngle
        set(value) {
            field = value.asDegrees.coerceIn(MINHARDSTOP.asDegrees, MAXHARDSTOP.asDegrees).degrees

            // For amp shot edge case
            if (!Robot.isAutonomous) Shooter.manualShootState = Shooter.manualShootState
            pivotMotor.setPositionSetpoint(angleSetpoint.asDegrees, 0.024 * (cos((pivotEncoderAngle + 20.0.degrees).asRadians)) /*+ 0.000001*/)

//            println("set pivot angle to $field")
        }

    val pivotError: Double
        get() = (pivotEncoderAngle - angleSetpoint).asDegrees.absoluteValue

    val distFromSpeaker: Double
        get() = if (AprilTag.aprilTagsEnabled) combinedPosition.distance(speakerPos.feet).asFeet else Drive.position.distance(speakerPos)

    init {
        stageAngleEntry.setDouble(20.0)

        pivotMotor.config {
            pid {
                p(0.00016)
                d(0.000004)
            }

            //                              ticks / gear ratio   fudge factor
            feedbackCoefficient = (360.0 / 2048.0 / GEARRATIO) * (107.0 / 305.0)
            coastMode()
            inverted(true)

            currentLimit(35, 40, 20)
        }

        pivotMotor.setRawOffset(pivotEncoderAngle.asDegrees)
        pivotAmpRate.setDouble(80.0)


        GlobalScope.launch {
            periodic {
                pivotCurrentEntry.setDouble(pivotMotor.current)
                ticksEntry.setDouble(pivotTicks.toDouble())
                encoderAngleEntry.setDouble(pivotEncoderAngle.asDegrees)
                motorAngleEntry.setDouble(pivotMotorAngle.asDegrees)
//                encoderVoltageEntry.setDouble(encoderVoltage)
                angleSetpointEntry.setDouble(angleSetpoint.asDegrees)

//                val pivotPos = Vector2(15.0, 6.0) - Vector2(15.0, 4.0).rotateDegrees(pivotEncoderAngle.asDegrees)
//                advantagePivotTransform = Transform3d(Translation3d(pivotPos.x.inches.asMeters, pivotPos.y.inches.asMeters, 0.0), Rotation3d((Math.PI / 2) + pivotEncoderAngle.asRadians, 0.0, (Math.PI / 2)))
//                advantagePivotPublisher.set(advantagePivotTransform)

                pivotMotor.setRawOffset(pivotEncoderAngle.asDegrees)

//                angleSetpoint = angleSetpoint

                distanceFromSpeakerEntry.setDouble(distFromSpeaker)

                if (aimSpeaker) {
                    angleSetpoint = if (OI.driverController.x) {
                        39.0.degrees
                    } else if (AprilTag.aprilTagsEnabled) {
                        Shooter.pitchCurve.getValue(distFromSpeaker + aimSpeakerDistanceOffset).degrees + angleFudge
                    } else {
                        PODIUMPOSE
                    }
//                    println("Angle: ${angle}")
                }
            }
        }

    }

    fun speakerIsReady(rpmTol: Double = 400.0, pitchTol: Double = 1.0, aimTol: Double = 3.0, debug: Boolean = false): Boolean {

        val rpmReady = abs(Shooter.rpmTopSetpoint - Shooter.motorRpmTop) < rpmTol && abs(Shooter.rpmBottomSetpoint - Shooter.motorRpmBottom) < rpmTol
        val pitchReady = abs(angleSetpoint.asDegrees - pivotEncoderAngle.asDegrees) < pitchTol
        val aimReady = abs(Drive.aimHeadingSetpoint.asDegrees - Drive.heading.asDegrees) < aimTol

        if (debug) {
            if (!rpmReady) println("Top RPM Setpoint: ${Shooter.rpmTopSetpoint} Top RPM: ${Shooter.motorRpmTop} Bottom RPM Setpoint: ${Shooter.rpmBottomSetpoint} Bottom RPM: ${Shooter.motorRpmBottom}")
            if (!pitchReady) println("Pitch Setpoint: ${angleSetpoint.asDegrees} Pitch: ${pivotEncoderAngle.asDegrees}")
            if (!aimReady) println("Aim Setpoint: ${Drive.aimHeadingSetpoint} Heading: ${Drive.heading}")
        }

        return rpmReady && pitchReady && aimReady
    }

    override fun postEnable() {
        pivotMotor.brakeMode()
    }

    override suspend fun default() {
        periodic {
            ticksEntry.setDouble(pivotTicks.toDouble())
        }
    }

    override fun onDisable() {
        pivotMotor.coastMode()
    }

    fun getAngleFromPosition(point: Vector2): Angle {
        return Shooter.pitchCurve.getValue(point.distance(speakerPos)).degrees
    }

}