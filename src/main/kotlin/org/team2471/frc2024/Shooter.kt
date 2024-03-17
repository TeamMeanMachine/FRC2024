package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.degrees

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
    private val topPEntry = table.getEntry("top P")
    private val topDEntry = table.getEntry("top D")
    private val bottomDEntry = table.getEntry("bottom D")
    private val bottomPEntry = table.getEntry("bottom P")
    private val shootingEntry = table.getEntry("shooting")
    val bottomAmpRPMEntry = table.getEntry("Bottom Amp RPM")
    val topAmpRPMEntry = table.getEntry("Top Amp RPM")
    val Pitch3Entry = table.getEntry("Pitch3Entry")
    val Pitch6Entry = table.getEntry("Pitch6Entry")
    val Pitch9Entry = table.getEntry("Pitch9Entry")
    val Pitch15Entry = table.getEntry("Pitch15Entry")
    val Pitch17Entry = table.getEntry("Pitch17Entry")
    val RPM3Entry = table.getEntry("RPM3Entry")
    val RPM6Entry = table.getEntry("RPM6Entry")
    val RPM9Entry = table.getEntry("RPM9Entry")
    val RPM15Entry = table.getEntry("RPM15Entry")
    val RPM17Entry = table.getEntry("RPM17Entry")


    val shooterMotorBottom = MotorController(FalconID(Falcons.SHOOTER_BOTTOM))
    val shooterMotorTop = MotorController(FalconID(Falcons.SHOOTER_TOP))

    val motorRpmTop
        get() = shooterMotorTop.velocity

    val motorRpmBottom
        get() = shooterMotorBottom.velocity

    var manualShootState = false
        set(value) {
            field = value
            if (!value && !Robot.isAutonomous) {
                rpmTopSetpoint = 0.0
                rpmBottomSetpoint = 0.0
            }
        }


    var kFeedForwardTop = 1.0/5300.0
    var kFeedForwardBottom = 1.0/5300.0

    val pitchCurve = MotionCurve()
    val rpmCurve = MotionCurve()

    private val topPDController = PDController(0.00015, 0.0002)
    private val bottomPDController = PDController(0.00015, 0.0002)

    private var ffTopPower: Double = 0.0
    private var ffBottomPower: Double = 0.0

    private var pdPowerTop = 0.0
    private var pdPowerBottom = 0.0

    const val MAXRPM = 5000.0
    var rpmTopSetpoint: Double = 0.0
        set(value) {
            val capped = value.coerceIn(0.0, MAXRPM)
            ffTopPower = capped * kFeedForwardTop
            if (capped == 0.0){
                pdPowerTop = 0.0
            }
            field = capped
        }
    var rpmBottomSetpoint: Double = 0.0
        set(value) {
            val capped = value.coerceIn(0.0, MAXRPM)
            ffBottomPower = capped * kFeedForwardBottom
            if (capped == 0.0){
                pdPowerBottom = 0.0
            }
            field = capped
        }

    init {

        if (!Pitch17Entry.exists()) {
            Pitch3Entry.setDouble(59.0)
            Pitch6Entry.setDouble(47.0)
            Pitch9Entry.setDouble(37.5)
            Pitch15Entry.setDouble(30.5)
            Pitch17Entry.setDouble(27.6)

            RPM3Entry.setDouble(3500.0)
            RPM6Entry.setDouble(3750.0)
            RPM9Entry.setDouble(5000.0)
            RPM15Entry.setDouble(5000.0)
            RPM17Entry.setDouble(5000.0)

            Pitch3Entry.setPersistent()
            Pitch6Entry.setPersistent()
            Pitch9Entry.setPersistent()
            Pitch15Entry.setPersistent()
            Pitch17Entry.setPersistent()
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
        topDEntry.setDouble(shooterMotorTop.getD())
        topPEntry.setDouble(shooterMotorTop.getP())
        bottomPEntry.setDouble(shooterMotorBottom.getP())
        bottomDEntry.setDouble(shooterMotorBottom.getD())

        if (isRedAlliance) {
            topAmpRPMEntry.setDouble(3000.0)
            bottomAmpRPMEntry.setDouble(3000.0)
        } else {
            topAmpRPMEntry.setDouble(3000.0)
            bottomAmpRPMEntry.setDouble(3000.0)
        }


        GlobalScope.launch {
            periodic {
                shooterCurrentEntry.setDouble(shooterMotorBottom.current)
//                if (rpmOne > 20.0) println("rpmOne = $rpmOne rpmTwo = $rpmTwo")
                shooterCurrentEntry.setDouble(shooterMotorBottom.current)
                shooterTwoCurrentEntry.setDouble(shooterMotorTop.current)
                motorRpmBottomEntry.setDouble(motorRpmBottom)
                motorRpmTopEntry.setDouble(motorRpmTop)
                shootingEntry.setBoolean(manualShootState)

//                println("entry: ${RPM3Entry.getDouble(5.0)}   curve: ${RPMCurve.getValue(3.0)}")
                if (Pitch3Entry.getDouble(3.0)!=pitchCurve.getValue(3.0)) { rebuildCurves() }
                if (Pitch6Entry.getDouble(6.0)!=pitchCurve.getValue(6.0)) { rebuildCurves() }
                if (Pitch9Entry.getDouble(9.0)!=pitchCurve.getValue(9.0)) { rebuildCurves() }
                if (Pitch15Entry.getDouble(13.7)!=pitchCurve.getValue(13.7)) { rebuildCurves() }
                if (Pitch17Entry.getDouble(17.0)!=pitchCurve.getValue(17.0)) { rebuildCurves() }
                if (RPM3Entry.getDouble(5.0)!=rpmCurve.getValue(3.0)) { rebuildCurves() }
                if (RPM6Entry.getDouble(6.0)!=rpmCurve.getValue(6.0)) { rebuildCurves() }
                if (RPM9Entry.getDouble(9.0)!=rpmCurve.getValue(9.0)) { rebuildCurves() }
                if (RPM15Entry.getDouble(13.7)!=rpmCurve.getValue(13.7)) { rebuildCurves() }
                if (RPM17Entry.getDouble(17.0)!=rpmCurve.getValue(17.0)) { rebuildCurves() }

                if (Robot.isEnabled || Robot.isAutonomous) {
                    if (rpmTopSetpoint == 0.0) {
                        shooterMotorTop.setPercentOutput(0.0)
                    } else {
                        pdPowerTop += topPDController.update(rpmTopSetpoint - motorRpmTop)
                        if (pdPowerTop + ffTopPower > 1.0) {
                            pdPowerTop = 1.0 - ffTopPower
                        }
                        shooterMotorTop.setPercentOutput(pdPowerTop + ffTopPower)
                    }

                    if (rpmBottomSetpoint == 0.0) {
                        shooterMotorBottom.setPercentOutput(0.0)
                    } else {
                        pdPowerBottom += bottomPDController.update(rpmBottomSetpoint - motorRpmBottom)
                        if (pdPowerBottom + ffBottomPower > 1.0) {
                            pdPowerBottom = 1.0 - ffBottomPower
                        }
                        shooterMotorBottom.setPercentOutput(pdPowerBottom + ffBottomPower)
                    }

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
            rpmTopEntry.setDouble(rpmTopSetpoint)
            rpmBottomEntry.setDouble(rpmBottomSetpoint)

            if (manualShootState) {
                // AMP SHOT!!!!!!!!!!!!!!!!!!!!! Bottom: 12 Top: 14!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Pivot Angle: 107.5
                // STAGE SHOT!!!!! Bottom 80: Top: 80   Pivot Angle: 32
                if (Pivot.pivotEncoderAngle > Pivot.CLOSESPEAKERPOSE + 5.0.degrees || Pivot.angleSetpoint > Pivot.CLOSESPEAKERPOSE + 5.0.degrees) {
                    rpmTopSetpoint = topAmpRPMEntry.getDouble(3000.0)
                    rpmBottomSetpoint = bottomAmpRPMEntry.getDouble(3000.0)
                } else if (Pivot.angleSetpoint == Pivot.CLOSESPEAKERPOSE) {
                    rpmTopSetpoint = 3500.0
                    rpmBottomSetpoint = 3500.0
                } else {
                    rpmTopSetpoint = rpmCurve.getValue(Drive.distance)
                    rpmBottomSetpoint = rpmCurve.getValue(Drive.distance)
                }
            }
        }
    }

    override fun preEnable() {
        rpmTopSetpoint  = 0.0
        rpmBottomSetpoint = 0.0
    }

    fun rebuildCurves() {
        println("Rebuilding curves. Hi.")
        pitchCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        pitchCurve.storeValue(3.0, Pitch3Entry.getDouble(61.0))
        pitchCurve.storeValue(6.0, Pitch6Entry.getDouble(50.0))
        pitchCurve.storeValue(9.0, Pitch9Entry.getDouble(42.0))
        pitchCurve.storeValue(13.7, Pitch15Entry.getDouble(30.0))
        pitchCurve.storeValue(17.0, Pitch17Entry.getDouble(28.0))

        rpmCurve.setMarkBeginOrEndKeysToZeroSlope(false)

        rpmCurve.storeValue(3.0, RPM3Entry.getDouble(3500.0))
        rpmCurve.storeValue(6.0, RPM6Entry.getDouble(3750.0))
        rpmCurve.storeValue(9.0, RPM9Entry.getDouble(5000.0))
        rpmCurve.storeValue(13.7, RPM15Entry.getDouble(5000.0))
        rpmCurve.storeValue(17.0, RPM17Entry.getDouble(5000.0))
    }

    fun setRpms(rpm: Double) {
        rpmTopSetpoint = rpm
        rpmBottomSetpoint = rpm
    }

    fun getRpmFromPosition(point: Vector2): Double {
        return rpmCurve.getValue(point.distance(Drive.speakerPos))
    }
}