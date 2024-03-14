package org.team2471.frc2024

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.AnalogInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
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
    private val encoderVoltageEntry = table.getEntry("Encoder Voltage")
    private val stageAngleEntry = table.getEntry("Stage Angle")
    private val distanceFromSpeakerEntry = table.getEntry("Distance From Speaker")
    var advantagePivotPublisher: StructPublisher<Transform3d> = NetworkTableInstance.getDefault().getStructTopic("Advantage Pivot Transform", Transform3d.struct).publish()


    val pivotMotor = MotorController(FalconID(Falcons.PIVOT))

    private val pivotEncoder = AnalogInput(AnalogSensors.PIVOT)

    private const val GEARRATIO = 1 / 61.71

    val TESTPOSE = 30.5.degrees //18 //32
    val CLOSESPEAKERPOSE = 59.0.degrees
    val MINHARDSTOP = 5.5.degrees
    val DRIVEPOSE = MINHARDSTOP + 2.0.degrees
    val MAXHARDSTOP = 110.2.degrees
    val AMPPOSE = 90.0.degrees//107.5.degrees

    // Ticks
    private val MINTICKS = if (isCompBot) 1924.0 else 2124.0
    private val MAXTICKS = if (isCompBot) 714.0 else 940.0

    var advantagePivotTransform = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d((Math.PI / 2) + MINHARDSTOP.asRadians, 0.0, (Math.PI / 2)))


    var aimSpeaker = false
        set(value) {
            field = value
            if (value) {
                Shooter.rpmTopSetpoint = Shooter.rpmCurve.getValue(distFromSpeaker)
                Shooter.rpmBottomSetpoint = Shooter.rpmTopSetpoint
            } else {
                Shooter.rpmTopSetpoint = 0.0
                Shooter.rpmBottomSetpoint = 0.0
            }
        }
    var revving = false

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
            Shooter.manualShootState = Shooter.manualShootState
//
        //            Uh oh
            pivotMotor.setPositionSetpoint(angleSetpoint.asDegrees, 0.024 * (cos((pivotEncoderAngle + 20.0.degrees).asRadians)) /*+ 0.000001*/)

//            println("set pivot angle to $field")
        }

    val pivotError: Double
        get() = (pivotEncoderAngle - angleSetpoint).asDegrees.absoluteValue

    val distFromSpeaker: Double
        get() = if (PoseEstimator.apriltagsEnabled) PoseEstimator.currentPose.distance(speakerPos) else AprilTag.last2DSpeakerDist.lastValue()





    init {
//        ticksOffsetEntry.setDouble(3665.0)
        stageAngleEntry.setDouble(20.0)

        pivotMotor.config {
            pid {
                p(0.00016)
                d(0.000004)
            }

            //                              ticks / gear ratio   fudge factor
            feedbackCoefficient = (360.0 / 2048.0 / GEARRATIO) * (107.0 / 305.0)
//            brakeMode()
            coastMode()
            inverted(true)

            currentLimit(35, 40, 20)
        }

        pivotMotor.setRawOffset(pivotEncoderAngle.asDegrees)


        GlobalScope.launch {
            periodic {
                pivotCurrentEntry.setDouble(pivotMotor.current)
                ticksEntry.setDouble(pivotTicks.toDouble())
                encoderAngleEntry.setDouble(pivotEncoderAngle.asDegrees)
                motorAngleEntry.setDouble(pivotMotorAngle.asDegrees)
                encoderVoltageEntry.setDouble(encoderVoltage)
                angleSetpointEntry.setDouble(angleSetpoint.asDegrees)

                advantagePivotTransform = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d((Math.PI / 2) + pivotEncoderAngle.asRadians, 0.0, (Math.PI / 2)))
                advantagePivotPublisher.set(advantagePivotTransform)


//                pivotErrorEntry.setDouble(pivotError)

                pivotMotor.setRawOffset(pivotEncoderAngle.asDegrees)

//                if (pivotError > 0.25) {
//                    pivotMotor.setPositionSetpoint(angleSetpoint.asDegrees, 0.024 * (cos((pivotEncoderAngle + 20.0.degrees).asRadians)) /*+ 0.000001*/)
////                    println(0.025 * cos((pivotEncoderAngle - 20.0.degrees).asRadians))
//                }

                distanceFromSpeakerEntry.setDouble(distFromSpeaker)

                if (aimSpeaker) {

                    // Calculated. May change a lot with more data
//                    val angle = (90.0 * (0.751492.pow(dist))).degrees

                    val angle = Shooter.pitchCurve.getValue(distFromSpeaker).degrees
//                    println("Angle: ${angle}")
                    angleSetpoint = angle
                }
            }
        }

    }

    fun speakerIsReady(rpmTol: Double = 400.0, pitchTol: Double = 1.0, aimTol: Double = 3.0, debug: Boolean = false): Boolean {
        if (revving) return false

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

    override fun onDisable() {
        pivotMotor.coastMode()
    }

}