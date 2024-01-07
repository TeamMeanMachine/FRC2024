package org.team2471.bunnybots2023

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.bunnybots2023.Limelight.toFieldCentric
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.control.PDConstantFController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller //Added by Jeremy on 1-30-23 for power testing
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.following.SwerveParameters
import org.team2471.frc.lib.units.*
import kotlin.math.absoluteValue
import kotlin.math.min

@OptIn(DelicateCoroutinesApi::class)
object Drive : Subsystem("Drive"), SwerveDrive {
    val robotHalfWidth = (30.0/2.0).inches
    val table = NetworkTableInstance.getDefault().getTable(name)
    val navXGyroEntry = table.getEntry("NavX Gyro")

    val odometer0Entry = table.getEntry("Odometer 0")
    val odometer1Entry = table.getEntry("Odometer 1")
    val odometer2Entry = table.getEntry("Odometer 2")
    val odometer3Entry = table.getEntry("Odometer 3")

    val absoluteAngle0Entry = table.getEntry("Analog Angle 0")
    val absoluteAngle1Entry = table.getEntry("Analog Angle 1")
    val absoluteAngle2Entry = table.getEntry("Analog Angle 2")
    val absoluteAngle3Entry = table.getEntry("Analog Angle 3")
    val motorAngle0Entry = table.getEntry("Motor Angle 0")
    val motorAngle1Entry = table.getEntry("Motor Angle 1")
    val motorAngle2Entry = table.getEntry("Motor Angle 2")
    val motorAngle3Entry = table.getEntry("Motor Angle 3")

    val turnMotor0CurrentEntry = table.getEntry("Turn Current 0")
    val turnMotor1CurrentEntry = table.getEntry("Turn Current 1")
    val turnMotor2CurrentEntry = table.getEntry("Turn Current 2")
    val turnMotor3CurrentEntry = table.getEntry("Turn Current 3")

    val driveMotor0CurrentEntry = table.getEntry("Drive Current 0")
    val driveMotor1CurrentEntry = table.getEntry("Drive Current 1")
    val driveMotor2CurrentEntry = table.getEntry("Drive Current 2")
    val driveMotor3CurrentEntry = table.getEntry("Drive Current 3")

    val totalDriveCurretEntry = table.getEntry("Total Drive Current")
    val totalTurnCurrentEntry = table.getEntry("Total Turn Current")

    val positionXEntry = table.getEntry("Position X")
    val positionYEntry = table.getEntry("Position Y")


    val useGyroEntry = table.getEntry("Use Gyro")

    val plannedPathEntry = table.getEntry("Planned Path")
    val actualRouteEntry = table.getEntry("Actual Route")

    val rateCurve = MotionCurve()


    override val parameters: SwerveParameters = SwerveParameters(
        gyroRateCorrection = 0.0,
        kpPosition = 0.32,
        kdPosition = 0.6,
        kPositionFeedForward = 0.05,
        kpHeading = 0.005,
        kdHeading = 0.02,
        kHeadingFeedForward = 0.001,
        kMoveWhileSpin = 0.0,
        invertDriveFactor = -1.0,
        invertSteerFactor = -1.0
    )


    /**
     * Coordinates of modules
     * **/
    override val modules: Array<SwerveDrive.Module> = arrayOf(
        Module(
            MotorController(SparkMaxID(Sparks.FRONT_LEFT_DRIVE)),
            MotorController(SparkMaxID(Sparks.FRONT_LEFT_STEER)),
            Vector2(-13.1, 13.1),
            Preferences.getDouble("Angle Offset 0",-259.95).degrees,
            DigitalSensors.FRONT_LEFT,
            odometer0Entry,
            0
        ),
        Module(
            MotorController(SparkMaxID(Sparks.FRONT_RIGHT_DRIVE)),
            MotorController(SparkMaxID(Sparks.FRONT_RIGHT_STEER)),
            Vector2(13.1, 13.1),
            Preferences.getDouble("Angle Offset 1",-351.13).degrees,
            DigitalSensors.FRONT_RIGHT,
            odometer1Entry,
            1
        ),
        Module(
            MotorController(SparkMaxID(Sparks.REAR_RIGHT_DRIVE)),
            MotorController(SparkMaxID(Sparks.REAR_RIGHT_STEER)),
            Vector2(13.1, -13.1),
            Preferences.getDouble("Angle Offset 2",-271.56).degrees,
            DigitalSensors.REAR_RIGHT,
            odometer2Entry,
            2
        ),
        Module(
            MotorController(SparkMaxID(Sparks.REAR_LEFT_DRIVE)),
            MotorController(SparkMaxID(Sparks.REAR_LEFT_STEER)),
            Vector2(-13.1, -13.1),
            Preferences.getDouble("Angle Offset 3",-231.51).degrees,
            DigitalSensors.REAR_LEFT,
            odometer3Entry,
            3
        )
    )

    private var navX: NavxWrapper = NavxWrapper()
    val gyro = navX
    private var gyroOffset = 0.0.degrees

    override var heading: Angle
        get() = (gyroOffset + gyro.angle.degrees).wrap()
        set(value) {
            gyro.reset()
            gyroOffset = -gyro.angle.degrees + value
        }

    override val headingRate: AngularVelocity
        get() = -gyro.rate.degrees.perSecond

    override var velocity = Vector2(0.0, 0.0)
    override var position = Vector2(0.0, 0.0)
    override val combinedPosition: Vector2
        get() = Vector2(0.0, 0.0)
    override var robotPivot = Vector2(0.0, 0.0)
    override var headingSetpoint = 0.0.degrees

    override val carpetFlow = Vector2(0.0, 1.0)
    override val kCarpet = 0.0104 // how much downstream and upstream carpet directions affect the distance, for no effect, use  0.0 (2.5% more distance downstream)
    override val kTread = 0.0 //.04 // how much of an effect treadWear has on distance (fully worn tread goes 4% less than full tread)  0.0 for no effect
    override val plannedPath: NetworkTableEntry = plannedPathEntry
    override val actualRoute: NetworkTableEntry = actualRouteEntry

    val autoPDController = PDConstantFController(0.015, 0.04, 0.05)
    val teleopPDController =  PDConstantFController(0.012, 0.09, 0.05)
    var aimPDController = teleopPDController

    var maxTranslation = 1.0
        get() =  if (demoMode) min(field, demoSpeed) else field
    var maxRotation = 0.8
        get() =  if (demoMode) min(field, demoSpeed) else field

    val isHumanDriving
        get() = OI.driveTranslation.length != 0.0 || OI.driveRotation != 0.0

    init {
        println("drive init")
        initializeSteeringMotors()

        GlobalScope.launch(MeanlibDispatcher) {
            println("in drive global scope")
            val headingEntry = table.getEntry("Heading")
            val xEntry = table.getEntry("X")
            val yEntry = table.getEntry("Y")
            val poseEntry = table.getEntry("advantageScopePose")

            SmartDashboard.setPersistent("Use Gyro")
            SmartDashboard.setPersistent("Gyro Type")

            if (!SmartDashboard.containsKey("DemoSpeed")) {
                println("DemoSpeed does not exist, setting it to 1.0")
                SmartDashboard.getEntry("DemoSpeed").setDouble(1.0)
                SmartDashboard.setPersistent("DemoSpeed")
            }

            navXGyroEntry.setBoolean(true)
            rateCurve.setMarkBeginOrEndKeysToZeroSlope(false)
            rateCurve.storeValue(1.0, 2.0)  // distance, rate
            rateCurve.storeValue(8.0, 6.0)  // distance, rate

            println("in init just before periodic")
            periodic {
                val (x, y) = position
                xEntry.setDouble(x)
                yEntry.setDouble(y)
                headingEntry.setDouble(heading.asDegrees)

                motorAngle0Entry.setDouble((modules[0] as Module).angle.wrap().asDegrees)
                motorAngle1Entry.setDouble((modules[1] as Module).angle.wrap().asDegrees)
                motorAngle2Entry.setDouble((modules[2] as Module).angle.wrap().asDegrees)
                motorAngle3Entry.setDouble((modules[3] as Module).angle.wrap().asDegrees)

                absoluteAngle0Entry.setDouble((modules[0] as Module).absoluteAngle.asDegrees)
                absoluteAngle1Entry.setDouble((modules[1] as Module).absoluteAngle.asDegrees)
                absoluteAngle2Entry.setDouble((modules[2] as Module).absoluteAngle.asDegrees)
                absoluteAngle3Entry.setDouble((modules[3] as Module).absoluteAngle.asDegrees)

                turnMotor0CurrentEntry.setDouble((modules[0] as Module).turnMotor.current)
                turnMotor1CurrentEntry.setDouble((modules[1] as Module).turnMotor.current)
                turnMotor2CurrentEntry.setDouble((modules[2] as Module).turnMotor.current)
                turnMotor3CurrentEntry.setDouble((modules[3] as Module).turnMotor.current)

                driveMotor0CurrentEntry.setDouble((modules[0] as Module).driveCurrent)
                driveMotor1CurrentEntry.setDouble((modules[1] as Module).driveCurrent)
                driveMotor2CurrentEntry.setDouble((modules[2] as Module).driveCurrent)
                driveMotor3CurrentEntry.setDouble((modules[3] as Module).driveCurrent)

                positionXEntry.setDouble(position.x)
                positionYEntry.setDouble(position.y)

                var totalDriveCurrent = 0.0
                for (i in modules) {
                    totalDriveCurrent += (i as Module).driveCurrent
                }
                totalDriveCurretEntry.setDouble(totalDriveCurrent)
                var totalTurnCurrent = 0.0
                for (i in modules) {
                    totalTurnCurrent += (i as Module).turnMotor.current
                }
                totalTurnCurrentEntry.setDouble(totalTurnCurrent)
            }
        }
    }

    override fun preEnable() {
        initializeSteeringMotors()
        odometer0Entry.setDouble(Preferences.getDouble("odometer 0",0.0))
        odometer1Entry.setDouble(Preferences.getDouble("odometer 1",0.0))
        odometer2Entry.setDouble(Preferences.getDouble("odometer 2",0.0))
        odometer3Entry.setDouble(Preferences.getDouble("odometer 3",0.0))
        println("prefs at enable=${Preferences.getDouble("odometer 0",0.0)}")
    }

    override fun onDisable() {
        if (odometer0Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 0", odometer0Entry.getDouble(0.0))
        if (odometer1Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 1", odometer1Entry.getDouble(0.0))
        if (odometer2Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 2", odometer2Entry.getDouble(0.0))
        if (odometer3Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 3", odometer3Entry.getDouble(0.0))
    }

    override fun poseUpdate(poseTwist: SwerveDrive.Pose) {
        //MAPoseEstimator.addDriveData(Timer.getFPGATimestamp(), Twist2d(poseTwist.position.y, poseTwist.position.x, -poseTwist.heading.asRadians))
    }

    fun zeroGyro() {
        heading = 0.0.degrees
        println("zeroed heading to $heading")//  alliance blue? ${AutoChooser.redSide}")
        Turret.rawTurretSetpoint = Turret.turretAngle.toFieldCentric()
    }

    override suspend fun default() {
        periodic {
            var turn = 0.0
            if (OI.driveRotation.absoluteValue > 0.001) {
                turn = OI.driveRotation
                if (!OI.driveLeftTriggerFullPress) {
                    turn = linearMap(0.0, 90.0, turn, turn * 0.5, Turret.turretError.asDegrees)
                }
            }

            if (!useGyroEntry.exists()) {
                useGyroEntry.setBoolean(true)
            }
            val useGyro2 = useGyroEntry.getBoolean(true) && !DriverStation.isAutonomous()
            drive(
                OI.driveTranslation * maxTranslation,
                turn * maxRotation,
                useGyro2,
                false
                )
            }
        }
    fun initializeSteeringMotors() {
        for (moduleCount in 0..3) { //changed to modules.indices, untested
            val module = (modules[moduleCount] as Module)
            module.turnMotor.setRawOffset(module.absoluteAngle.asDegrees)
            println("Module: $moduleCount analogAngle: ${module.absoluteAngle} id: ${module.driveMotor.motorID}")
        }
    }

    fun resetDriveMotors() {
        for (moduleCount in 0..3) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.restoreFactoryDefaults()
            println("For module $moduleCount, drive motor's factory defaults were restored.")
        }
    }

    fun resetSteeringMotors() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.turnMotor.restoreFactoryDefaults()
            println("For module $moduleCount, turn motor's factory defaults were restored.")
        }
    }

    fun brakeMode() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.brakeMode()
        }
    }

    fun coastMode() {
        for (element in modules) { //switched from element in 0..modules.size-1, untested
            val module = (element as Module)
            module.driveMotor.coastMode()
            module.turnMotor.coastMode()
        }
    }

    class Module(
        val driveMotor: MotorController,
        val turnMotor: MotorController,
        override val modulePosition: Vector2,
        override var angleOffset: Angle,
        digitalInputID: Int,
        private val odometerEntry: NetworkTableEntry,
        val index: Int
    ) : SwerveDrive.Module {
        companion object {
            private const val ANGLE_MAX = 983
            private const val ANGLE_MIN = 47

            private val P = 0.0075 //0.010
            private val D = 0.00075
        }

        override val angle: Angle
            get() = turnMotor.position.degrees * parameters.invertSteerFactor

        val digitalEncoder : DutyCycleEncoder = DutyCycleEncoder(digitalInputID)

        val absoluteAngle: Angle
            get() {
                return (-digitalEncoder.absolutePosition.degrees * 360.0 - angleOffset).wrap()
            }

        override val treadWear: Double
            get() = linearMap(0.0, 10000.0, 1.0, 0.96, odometer).coerceIn(0.96, 1.0)

        val driveCurrent: Double
            get() = driveMotor.current

        private val pdController = PDController(P, D)

        override val speed: Double
            get() = driveMotor.velocity

        val power: Double
            get() {
                return driveMotor.output * parameters.invertDriveFactor
            }

        override val currDistance: Double
            get() = driveMotor.position * parameters.invertDriveFactor

        override var prevDistance: Double = 0.0

        override var odometer: Double
            get() = odometerEntry.getDouble(0.0)
            set(value) { odometerEntry.setDouble(value) }

        override fun zeroEncoder() {
            driveMotor.position = 0.0
        }

        override var angleSetpoint: Angle = 0.0.degrees
            set(value) = turnMotor.setPositionSetpoint(value.unWrap(angle).asDegrees * parameters.invertSteerFactor)

        override fun setDrivePower(power: Double) {
            driveMotor.setPercentOutput(power * parameters.invertDriveFactor)
        }


        val error: Angle
            get() = turnMotor.closedLoopError.degrees

        init {
            println("Drive.module.init")
            print(angle.asDegrees)
            turnMotor.config {
                feedbackCoefficient = (360.0 / 42.0 / 12.0 / 5.08) * (360.5 / 274.04)
                inverted(false)
                setSensorPhase(false)
                coastMode()
                println("Absolute Angle: ${absoluteAngle.asDegrees}")
                setRawOffsetConfig(absoluteAngle.asDegrees)
                currentLimit(12, 16, 1)
                pid {
                    p(0.0002)
//                    d(0.0000025)
                }
            }
            driveMotor.config {
                brakeMode()
                //                    wheel diam / 12 in per foot * pi / ticks / gear ratio
                feedbackCoefficient = 3.0 / 12.0 * Math.PI / 42.0 / 5.5
                currentLimit(25, 30, 1)
                openLoopRamp(0.2)
            }
            GlobalScope.launch {
                periodic {
//                    println("${turnMotor.motorID}   ${ round(absoluteAngle.asDegrees, 2) }")
                }
            }
        }

        override fun driveWithDistance(angle: Angle, distance: Length) {
            driveMotor.setPositionSetpoint(distance.asFeet)
            val error = (angle - this.angle).wrap()
            pdController.update(error.asDegrees)
        }

        override fun stop() {
            driveMotor.stop()
        }

        fun setAngleOffset() {
            val digitalAngle = -digitalEncoder.absolutePosition
            angleOffset = digitalAngle.degrees * 360.0
            Preferences.setDouble("Angle Offset $index", angleOffset.asDegrees)
            println("Angle Offset $index = $digitalAngle")
        }
    }
    fun setAngleOffsets() {
        for (element in modules) {
            val module = (element as Module)
            module.setAngleOffset()
        }
        initializeSteeringMotors()
    }
}

fun Drive.abortPath(): Boolean {
    return isHumanDriving
}

suspend fun Drive.currentTest() = use(this) {
    var power = 0.0
    var upPressed = false
    var downPressed = false
    periodic {
        if (OI.driverController.dPad == Controller.Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == Controller.Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != Controller.Direction.UP && upPressed) {
            upPressed = false
            power += 0.001
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            power -= 0.001
        }
//        for (moduleCount in 0..3) {
//            val module = modules[moduleCount] as Drive.Module
//        }
//        println()
//        println("power: $power")
        var currModule = modules[0] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[1] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[2] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[3] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        
        println("current: ${round(currModule.driveCurrent, 2)}  power: $power")
    //    val currModule2 = modules[3] as Drive.Module
      //  currModule2.driveMotor.setPercentOutput(power)
        //currModule2.turnMotor.setPositionSetpoint(0.0)
       // println("current: ${round(currModule.driveCurrent, 2)}  power: $power")

    //        drive(
//            Vector2(0.0, power),
//            0.0,
//            false
//        )
    }
}