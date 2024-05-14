package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.degrees
import org.team2471.frc2024.Drive.isRedAlliance

object Shooter: Subsystem("Shooter") {

    private val table = NetworkTableInstance.getDefault().getTable("Shooter")

    private val shooterPercentEntry = table.getEntry("Shooter Percent")
    private val shooterCurrentEntry = table.getEntry("Shooter Current")
    private val shooterTwoCurrentEntry = table.getEntry("Shooter Two Current")
    private val motorRpmBottomEntry = table.getEntry("RPM Bottom")
    private val motorRpmTopEntry = table.getEntry("RPM Top")
    private val rpmTopEntry = table.getEntry("rpm top setpoint")
    private val rpmBottomEntry = table.getEntry("rpm bottom setpoint")
    private val shootingRpmTopEntry = table.getEntry("Shooting RPM Top")
    private val shootingRpmBottomEntry = table.getEntry("Shooting RPM Bottom")
    private val shootingEntry = table.getEntry("shooting")
    val bottomAmpRPMEntry = table.getEntry("Bottom Amp RPM")
    val topAmpRPMEntry = table.getEntry("Top Amp RPM")
    val Pitch3_5Entry = table.getEntry("Pitch3.5Entry")
    val Pitch5Entry = table.getEntry("Pitch5Entry")
    val Pitch7Entry = table.getEntry("Pitch7Entry")
    val Pitch9Entry = table.getEntry("Pitch9Entry")
    val Pitch11Entry = table.getEntry("Pitch11Entry")
    val Pitch13Entry = table.getEntry("Pitch13Entry")
    val Pitch15Entry = table.getEntry("Pitch15Entry")
    val Pitch17Entry = table.getEntry("Pitch17Entry")
//    val Pitch19Entry = table.getEntry("Pitch19Entry")
//    val Pitch21Entry = table.getEntry("Pitch21Entry")

    val RPM3Entry = table.getEntry("RPM3Entry")
    val RPM6Entry = table.getEntry("RPM6Entry")
    val RPM9Entry = table.getEntry("RPM9Entry")
    val RPM15Entry = table.getEntry("RPM15Entry")
    val RPM17Entry = table.getEntry("RPM17Entry")

    val demoRPMEntry = table.getEntry("DemoRPM")

    const val NEG_POWER = -0.001 //min for falcon to even consider
    const val MAXRPM = 5800.0

    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP))

    val motorRpmTop
        get() = shooterMotorTop.velocity


    val motorRpmBottom
        get() = shooterMotorBottom.velocity

    @OptIn(DelicateCoroutinesApi::class)
    var manualShootState = false
        set(value) {
            field = value
            if (!value && !Robot.isAutonomous) {
                rpmTopSetpoint = 0.0
                rpmBottomSetpoint = 0.0
            }
        }

    var kFeedForwardTop = 1.0/5800.0
    var kFeedForwardBottom = 1.0/5800.0

    val pitchCurve = MotionCurve()
    val rpmCurve = MotionCurve()

    private val topPDController = PDController(0.0002, 0.0002)
    private val bottomPDController = PDController(0.0002, 0.0002)

    private var ffTopPower: Double = 0.0
    private var ffBottomPower: Double = 0.0

    private var pdPowerTop = 0.0
    private var pdPowerBottom = 0.0

    var rpmTopSetpoint: Double = 0.0
        set(value) {
            val capped = value.coerceIn(0.0, MAXRPM)
            ffTopPower = if (capped == 0.0) -0.2 * kFeedForwardTop * motorRpmTop else capped * kFeedForwardTop //only ff going up
            field = capped
        }
    var rpmBottomSetpoint: Double = 0.0
        set(value) {
            val capped = value.coerceIn(0.0, MAXRPM)
            ffBottomPower = if (capped == 0.0) -0.2 * kFeedForwardBottom * motorRpmBottom else capped * kFeedForwardBottom //only ff going up
            field = capped
        }

    init {
        demoRPMEntry.setDouble(2500.0)

//        if (!Robot.inComp) {
            if (!Pitch17Entry.exists() || !Pitch3_5Entry.exists()) {
                Pitch3_5Entry.setDouble(57.0)
                Pitch5Entry.setDouble(52.0)
                Pitch7Entry.setDouble(43.0)
                Pitch9Entry.setDouble(38.0)
                Pitch11Entry.setDouble(34.0)
                Pitch13Entry.setDouble(31.0)
                Pitch15Entry.setDouble(30.0)
                Pitch17Entry.setDouble(29.0)
//                Pitch19Entry.setDouble(0.0)
//                Pitch21Entry.setDouble(0.0)

                RPM3Entry.setDouble(3500.0)
                RPM6Entry.setDouble(3750.0)
                RPM9Entry.setDouble(5000.0)
                RPM15Entry.setDouble(5000.0)
                RPM17Entry.setDouble(5000.0)

                Pitch3_5Entry.setPersistent()
                Pitch5Entry.setPersistent()
                Pitch7Entry.setPersistent()
                Pitch9Entry.setPersistent()
                Pitch11Entry.setPersistent()
                Pitch13Entry.setPersistent()
                Pitch15Entry.setPersistent()
                Pitch17Entry.setPersistent()
//                Pitch19Entry.setPersistent()
//                Pitch21Entry.setPersistent()
                RPM3Entry.setPersistent()
                RPM6Entry.setPersistent()
                RPM9Entry.setPersistent()
                RPM15Entry.setPersistent()
                RPM17Entry.setPersistent()
            }

            shooterPercentEntry.setDouble(1.0)
            if (!shootingRpmTopEntry.exists()) {
                shootingRpmTopEntry.setDouble(5000.0)
                shootingRpmTopEntry.setPersistent()
            }

            if (!shootingRpmBottomEntry.exists()) {
                shootingRpmBottomEntry.setDouble(5000.0)
                shootingRpmBottomEntry.setPersistent()
            }

            if (isRedAlliance) {
                topAmpRPMEntry.setDouble(2000.0)
                bottomAmpRPMEntry.setDouble(2000.0)
            } else {
                topAmpRPMEntry.setDouble(2000.0)
                bottomAmpRPMEntry.setDouble(2000.0)
            }
//        }

        shooterMotorBottom.config {
            feedbackCoefficient = 53.0 * (400.0 / 350.0)
            currentLimit(30, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }

        shooterMotorTop.config {
            feedbackCoefficient = 53.0 * (400.0 / 350.0)
            currentLimit(30, 40, 1)
            coastMode()
            inverted(true)
            followersInverted(true)
        }


        GlobalScope.launch {
            periodic {
//                if (rpmOne > 20.0) println("rpmOne = $rpmOne rpmTwo = $rpmTwo")
                shooterCurrentEntry.setDouble(shooterMotorBottom.current)
                shooterTwoCurrentEntry.setDouble(shooterMotorTop.current)
                motorRpmBottomEntry.setDouble(motorRpmBottom)
                motorRpmTopEntry.setDouble(motorRpmTop)
                shootingEntry.setBoolean(manualShootState)
                rpmTopEntry.setDouble(rpmTopSetpoint)
                rpmBottomEntry.setDouble(rpmBottomSetpoint)

//                println("entry: ${RPM3Entry.getDouble(5.0)}   curve: ${RPMCurve.getValue(3.0)}")

//                if (!inComp) {
                if (Pitch3_5Entry.getDouble(0.0)!=pitchCurve.getValue(3.5)) rebuildCurves()
                if (Pitch5Entry.getDouble(0.0)!=pitchCurve.getValue(5.0)) rebuildCurves()
                if (Pitch7Entry.getDouble(0.0)!=pitchCurve.getValue(7.0)) rebuildCurves()
                if (Pitch9Entry.getDouble(0.0)!=pitchCurve.getValue(9.0)) rebuildCurves()
                if (Pitch11Entry.getDouble(0.0)!=pitchCurve.getValue(11.0)) rebuildCurves()
                if (Pitch13Entry.getDouble(0.0)!=pitchCurve.getValue(13.0)) rebuildCurves()
                if (Pitch15Entry.getDouble(0.0)!=pitchCurve.getValue(15.0)) rebuildCurves()
                if (Pitch17Entry.getDouble(0.0)!=pitchCurve.getValue(17.0)) rebuildCurves()
//                if (Pitch19Entry.getDouble(0.0)!=pitchCurve.getValue(19.0)) rebuildCurves()
//                if (Pitch21Entry.getDouble(0.0)!=pitchCurve.getValue(21.0)) rebuildCurves()
                if (RPM3Entry.getDouble(5.0)!=rpmCurve.getValue(3.0)) rebuildCurves()
                if (RPM6Entry.getDouble(6.0)!=rpmCurve.getValue(6.0)) rebuildCurves()
                if (RPM9Entry.getDouble(9.0)!=rpmCurve.getValue(9.0)) rebuildCurves()
                if (RPM15Entry.getDouble(13.7)!=rpmCurve.getValue(13.7)) rebuildCurves()
                if (RPM17Entry.getDouble(17.0)!=rpmCurve.getValue(17.0)) rebuildCurves()
//                }

                if (Robot.isEnabled || Robot.isAutonomous) {
                    pdPowerTop += topPDController.update(rpmTopSetpoint - motorRpmTop)
                    var power = pdPowerTop + ffTopPower
                    if (rpmTopSetpoint == 0.0) { // ramp down
                        pdPowerTop = 0.0
                        power = if (motorRpmTop > 1.0 && motorRpmTop < 4000.0
                            && (Intake.intakeState == Intake.IntakeState.INTAKING || Intake.intakeState == Intake.IntakeState.SLOWING))
                            NEG_POWER else 0.0
                        power = power.coerceIn(NEG_POWER, 1.0)
                    } else {
                        if (pdPowerTop + ffTopPower > 1.0) pdPowerTop = 1.0 - ffTopPower
                    }
                    shooterMotorTop.setPercentOutput(power)


                    pdPowerBottom += bottomPDController.update(rpmBottomSetpoint - motorRpmBottom)
                    power = pdPowerBottom + ffBottomPower
                    if (rpmBottomSetpoint == 0.0) { // ramp down
                        pdPowerBottom = 0.0
                        power = if (motorRpmBottom > 1.0 && motorRpmBottom < 4000.0
                            && (Intake.intakeState == Intake.IntakeState.INTAKING || Intake.intakeState == Intake.IntakeState.SLOWING))
                            NEG_POWER else 0.0
                        power = power.coerceIn(NEG_POWER, 1.0)
                    } else {
                        if (pdPowerBottom + ffBottomPower > 1.0) pdPowerBottom = 1.0 - ffBottomPower
                    }
                    shooterMotorBottom.setPercentOutput(power)

//                    println("topPower: $ffTopPower   bottomPower: $ffBottomPower")
                }

//                if (Robot.isAutonomousEnabled && !Pivot.revving) { Shoots out too early during staging too often
////                    println("Pre-Revving!!!")
//                    rpmTopSetpoint = 5000.0
//                    rpmBottomSetpoint = 5000.0
////                    println("RpmSetpoints: Top: $rpmTopSetpoint Bottom: $rpmBottomSetpoint")
//                }
            }
        }
    }

    override suspend fun default() {
        periodic {
            if (manualShootState) {
                // AMP SHOT!!!!!!!!!!!!!!!!!!!!! Bottom: 12 Top: 14!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pivot Angle: 107.5
                // STAGE SHOT!!!!! Bottom 80: Top: 80   Pivot Angle: 32
                if (Pivot.pivotEncoderAngle > Pivot.CLOSESPEAKERPOSE + 5.0.degrees || Pivot.angleSetpoint > Pivot.CLOSESPEAKERPOSE + 5.0.degrees) {
                    rpmTopSetpoint = topAmpRPMEntry.getDouble(1200.0)
                    rpmBottomSetpoint = bottomAmpRPMEntry.getDouble(1200.0)
                } else if (Pivot.angleSetpoint == Pivot.CLOSESPEAKERPOSE) {
                    rpmTopSetpoint = 3500.0
                    rpmBottomSetpoint = 3500.0
                } else if (Drive.demoMode && Pivot.angleSetpoint == Pivot.demoAngleEntry.getDouble(Pivot.DEMO_POSE.asDegrees).degrees) {
                    rpmTopSetpoint = demoRPMEntry.getDouble(2500.0)
                    rpmBottomSetpoint = demoRPMEntry.getDouble(2500.0)
                } else {
                    if (AprilTag.aprilTagsEnabled) {
                        rpmTopSetpoint = rpmCurve.getValue(Pivot.distFromSpeaker)
                        rpmBottomSetpoint = rpmCurve.getValue(Pivot.distFromSpeaker)
                    } else {
                        rpmTopSetpoint = 5000.0
                        rpmBottomSetpoint = 5000.0
                    }
                }
            }
        }
    }

    override fun postEnable() {
        println("inside shooter preEnable ${Robot.totalTimeTaken()}")
        GlobalScope.launch {
            rpmTopSetpoint  = 0.0
            rpmBottomSetpoint = 0.0
        }
        println("after shooter preEnable ${Robot.totalTimeTaken()}")
    }

    fun rebuildCurves() {
        pitchCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        pitchCurve.storeValue(4.2, Pitch3_5Entry.getDouble(70.0))
        pitchCurve.storeValue(5.0, Pitch5Entry.getDouble(70.0))
        pitchCurve.storeValue(7.0, Pitch7Entry.getDouble(70.0))
        pitchCurve.storeValue(9.0, Pitch9Entry.getDouble(70.0))
        pitchCurve.storeValue(11.0, Pitch11Entry.getDouble(70.0))
        pitchCurve.storeValue(13.0, Pitch13Entry.getDouble(70.0))
        pitchCurve.storeValue(15.0, Pitch15Entry.getDouble(70.0))
        pitchCurve.storeValue(17.0, Pitch17Entry.getDouble(70.0))

        rpmCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        rpmCurve.storeValue(3.0, RPM3Entry.getDouble(3500.0))
        rpmCurve.storeValue(6.0, RPM6Entry.getDouble(3750.0))
        rpmCurve.storeValue(9.0, RPM9Entry.getDouble(5800.0))
        rpmCurve.storeValue(13.7, RPM15Entry.getDouble(5800.0))
        rpmCurve.storeValue(17.0, RPM17Entry.getDouble(5800.0))

//        println("3.5: ${pitchCurve.getValue(3.5)}")
//        println("5.0: ${pitchCurve.getValue(5.0)}")
//        println("6.0: ${pitchCurve.getValue(6.0)}")
//        println("7.0: ${pitchCurve.getValue(7.0)}")
//        println("9.0: ${pitchCurve.getValue(9.0)}")
//        println("11.0: ${pitchCurve.getValue(11.0)}")
//        println("13.0: ${pitchCurve.getValue(13.0)}")
//        println("15.0: ${pitchCurve.getValue(15.0)}")
//        println("17.0: ${pitchCurve.getValue(17.0)}")

//        println("1: ${pitchCurve.getValue(3.7 - 7.0.inches.asFeet).round(1)} 2: ${pitchCurve.getValue(5.0 - 7.0.inches.asFeet).round(1)} 3: ${pitchCurve.getValue(7.0 - 7.0.inches.asFeet).round(1)} 4: ${pitchCurve.getValue(9.0 - 7.0.inches.asFeet).round(1)} 5: ${pitchCurve.getValue(11.0 - 7.0.inches.asFeet).round(1)} 6: ${pitchCurve.getValue(13.0 - 7.0.inches.asFeet).round(1)} 7: ${pitchCurve.getValue(13.7 - 7.0.inches.asFeet).round(1)} 8: ${pitchCurve.getValue(15.0 - 7.0.inches.asFeet).round(1)} 9: ${pitchCurve.getValue(17.0 - 7.0.inches.asFeet).round(1)}")

    }

    fun setRpms(rpm: Double) {
        rpmTopSetpoint = rpm
        rpmBottomSetpoint = rpm
    }

    fun getRpmFromPosition(point: Vector2): Double {
        return rpmCurve.getValue(point.distance(Drive.speakerPos))
    }
}