package org.team2471.frc2024

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.control.PDConstantFController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller //Added by Jeremy on 1-30-23 for power testing
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.following.SwerveParameters
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.Timer
import org.team2471.frc2024.Drive.advantageWheelPoseEntry
import org.team2471.frc2024.Drive.combinedPosition
import org.team2471.frc2024.Drive.deltaPos
import org.team2471.frc2024.Drive.heading
import org.team2471.frc2024.Drive.position
import org.team2471.frc2024.Drive.prevCombinedPosition
import org.team2471.frc2024.Drive.testWheelPosition
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.pow

@OptIn(DelicateCoroutinesApi::class)
object Drive : Subsystem("Drive"), SwerveDrive {
    val robotHalfWidth = (25.0/2.0).inches

//    const val turnZero0 = -34.0
//    const val turnZero1 = 321.5
//    const val turnZero2 = 21.7
//    const val turnZero3 = 17.2

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

    val drivePowerEntry = table.getEntry("Drive Percent Out")

    val totalDriveCurretEntry = table.getEntry("Total Drive Current")
    val totalTurnCurrentEntry = table.getEntry("Total Turn Current")

    val positionXEntry = table.getEntry("Position X")
    val positionYEntry = table.getEntry("Position Y")

    val speedEntry = table.getEntry("Speed")
    val accelerationEntry = table.getEntry("Acceleration")
    val rotationalSpeedEntry = table.getEntry("rotational Speed")
    val rotationalAccelerationEntry = table.getEntry("rotational Acceleration")

    val useGyroEntry = table.getEntry("Use Gyro")

    val plannedPathEntry = table.getEntry("Planned Path")
    val actualRouteEntry = table.getEntry("Actual Route")
    val distanceEntry = table.getEntry("Distance From Speaker Drive")

    private val advantagePoseEntry = table.getEntry("Drive Advantage Pose")

    val advantageWheelPoseEntry = table.getEntry("Test Wheel Advantage Pose")

    private val advantageCombinedPoseEntry = table.getEntry("Combined Advantage Pose")


    val rateCurve = MotionCurve()


    override val parameters: SwerveParameters = SwerveParameters(
        gyroRateCorrection = 0.0,
        kpPosition = 0.32,
        kdPosition = 0.6,
        kPositionFeedForward = 0.05,
        kpHeading = 0.005,
        kdHeading = 0.02,
        kHeadingFeedForward = 0.001,
        kMoveWhileSpin = 27.0,
        invertDriveFactor = 1.0,
        invertSteerFactor = -1.0
    )


    /**
     * Coordinates of modules
     * **/
    override val modules: Array<SwerveDrive.Module> = arrayOf(
        Module(
            MotorController(FalconID(Falcons.FRONT_LEFT_DRIVE)),
            MotorController(SparkMaxID(Sparks.FRONT_LEFT_STEER)),
            Vector2(-10.75, 10.75),
            Preferences.getDouble("Angle Offset 0",-137.6).degrees,
            DigitalSensors.FRONT_LEFT,
            odometer0Entry,
            0
        ),
        Module(
            MotorController(FalconID(Falcons.FRONT_RIGHT_DRIVE)),
            MotorController(SparkMaxID(Sparks.FRONT_RIGHT_STEER)),
            Vector2(10.75, 10.75),
            Preferences.getDouble("Angle Offset 1",-21.4).degrees,
            DigitalSensors.FRONT_RIGHT,
            odometer1Entry,
            1
        ),
        Module(
            MotorController(FalconID(Falcons.BACK_RIGHT_DRIVE)),
            MotorController(SparkMaxID(Sparks.BACK_RIGHT_STEER)),
            Vector2(10.75, -10.75),
            Preferences.getDouble("Angle Offset 2",37.1).degrees,
            DigitalSensors.BACK_RIGHT,
            odometer2Entry,
            2
        ),
        Module(
            MotorController(FalconID(Falcons.BACK_LEFT_DRIVE)),
            MotorController(SparkMaxID(Sparks.BACK_LEFT_STEER)),
            Vector2(-10.75, -10.75),
            Preferences.getDouble("Angle Offset 3",165.6).degrees,
            DigitalSensors.BACK_LEFT,
            odometer3Entry,
            3
        )
    )

    private var navX: NavxWrapper = NavxWrapper()
    val gyro = navX
    private var gyroOffset = 0.0.degrees

    override var heading: Angle
        get() = (gyroOffset - gyro.angle.degrees).wrap()
        set(value) {
            gyro.reset()
            gyroOffset = gyro.angle.degrees + value
        }

    private var prevSpeed = 0.0
    private var prevRotationalSpeed = 0.0
    private var prevHeading = 0.0
    private var prevPosition = Vector2(0.0, 0.0)
    private var prevTime = 0.0
//    private var velocityField = Vector2(0.0, 0.0)

    override val headingRate: AngularVelocity
        get() = -gyro.rate.degrees.perSecond

    override var velocity = Vector2(0.0, 0.0)
    override var position = Vector2(0.0, 0.0)
        set(value) {
            field = value
        }
    override var deltaPos = Vector2L(0.0.inches, 0.0.inches)

    // velocity over 0.02 seconds
    var tickVelocity = Vector2(0.0, 0.0)
    var prevTickVelocity = Vector2(0.0, 0.0)

    override var combinedPosition: Vector2L = position.feet
    var prevCombinedPosition: Vector2L = position.feet


    var testWheelPosition: Vector2L = position.feet
//                   feet seconds fps fudge
    val driveStDevM = (2.5 / 30 / 50 * 15).feet.asMeters


    override var robotPivot = Vector2(0.0, 0.0)
    override var headingSetpoint = 0.0.degrees

    override val carpetFlow = Vector2(1.0, 0.0) //STEM?
    //    override val carpetFlow = Vector2(-1.0, 0.0) //Salem?
//    override val carpetFlow = Vector2(0.0, 1.0) //2023 pre-wpi field x y swap?
    override val kCarpet = 0.0212 //0.052 // how much downstream and upstream carpet directions affect the distance, for no effect, use  0.0 (2.12% more distance downstream)
    override val kTread = 0.035 //.04 // how much of an effect treadWear has on distance (fully worn tread goes 4% less than full tread)  0.0 for no effect
    override val plannedPath: NetworkTableEntry = plannedPathEntry
    override val actualRoute: NetworkTableEntry = actualRouteEntry

    val autoPDController = PDConstantFController(0.015, 0.04, 0.02)
    val teleopPDController =  PDConstantFController( 0.01,0.075, 0.025)
//    val teleopPDController =  PDConstantFController(parameters.kpHeading, parameters.kdHeading, parameters.kHeadingFeedForward)

    var aimPDController = teleopPDController

    var aimSpeaker = false

    var aimAmp = false
    val speakerPos
        get() = if (isRedAlliance) Vector2(642.73.inches.asFeet, 223.42.inches.asFeet) else Vector2(8.5.inches.asFeet, 213.42.inches.asFeet) //orig 218.42 for both -- aiming left

    val ampPos = Vector2(0.0, 0.0) //TODO

    var aimHeadingSetpoint = 0.0.radians
    val distanceFromSpeakerDrivePos: Double
        get() = (position - speakerPos).length

    var maxTranslation = 1.0
        get() =  if (demoMode) min(field, demoSpeed) else field
    var maxRotation = 1.0
        get() =  if (demoMode) min(field, demoSpeed) else field

    val isHumanDriving
        get() = OI.driveTranslation.length != 0.0 || OI.driveRotation != 0.0

    val fieldDimensionsInMeters = Vector2(26.29.feet.asMeters,54.27.feet.asMeters) // field diagram & json is 26.29, 54.27 but includes side walls and barriers
    val fieldCenterOffsetInMeters = fieldDimensionsInMeters/2.0

    val isRedAlliance: Boolean
        get() {
            if (DriverStation.getAlliance().isEmpty) {
                return true
            } else {
                return DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            }
        }

    val isBlueAlliance: Boolean
        get() = !isRedAlliance

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
            val t = Timer()
            t.start()
            periodic {
                val batteryVolt = RobotController.getBatteryVoltage() > 12.7
                SmartDashboard.putBoolean("Battery Good", batteryVolt)
                val demoDisabled = demoSpeed == 1.0
                SmartDashboard.putBoolean("FullSpeed", demoDisabled)

                val (x, y) = position
                xEntry.setDouble(x)
                yEntry.setDouble(y)
                headingEntry.setDouble(heading.asDegrees)

                advantagePoseEntry.setDoubleArray(
                    doubleArrayOf(
                        position.x.feet.asMeters,
                        position.y.feet.asMeters,
                        heading.asDegrees
                    )
                )

                advantageCombinedPoseEntry.setAdvantagePose(combinedPosition, heading)

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

                drivePowerEntry.setDouble((modules[0] as Module).power)

                positionXEntry.setDouble(position.x)
                positionYEntry.setDouble(position.y)
                distanceEntry.setDouble(distanceFromSpeakerDrivePos)

                val time = t.get()
                val dt = time - prevTime
                velocity = (position - prevPosition) / dt

                prevTickVelocity = tickVelocity
                tickVelocity = position - prevPosition


//                val speed = velocity.length
//                speedEntry.setDouble(speed)
//                val acceleration = (speed - prevSpeed) / dt
//                accelerationEntry.setDouble(acceleration)
//                val rotationalSpeed = (heading.asDegrees - prevHeading) / dt
//                rotationalSpeedEntry.setDouble(rotationalSpeed)
//                val rotationalAcceleration = (rotationalSpeed - prevRotationalSpeed) / dt
//                rotationalAccelerationEntry.setDouble(rotationalAcceleration)
//                if (speed.round(2) != 0.0) println("t: ${time.round(2)}  position: ${position.round(2)}  speed: ${speed.round(2)}  accel: ${acceleration.round(2)}  heading ${heading.asDegrees.round(2)}  rSpeed: ${rotationalSpeed.round(2)}  rAccel: ${rotationalAcceleration.round(2)}")
//                prevSpeed = speed
//                prevRotationalSpeed = rotationalSpeed
//                prevHeading = heading.asDegrees
                prevPosition = position
                prevTime = time

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

                updatePos(driveStDevM, *AprilTag.getCurrentGlobalPoses())


                if (Robot.isAutonomous && aimSpeaker) {
                    var turn = 0.0
                    val aimTurn = aimSpeakerAmpLogic()

                    if (aimTurn != null) {
                        turn = aimTurn
                    }

                    if (!useGyroEntry.exists()) {
                        useGyroEntry.setBoolean(true)
                    }
                    val useGyro2 = useGyroEntry.getBoolean(true) && !DriverStation.isAutonomous()

                    drive(
                        OI.driveTranslation * maxTranslation,
                        turn * maxRotation,
                        useGyro2,
                        !aimSpeaker  // true for teleop, false when aiming
                    )
                }
            }
        }
    }

    override fun preEnable() {
//        Preferences.setDouble("odometer 0", 0.0)
//        Preferences.setDouble("odometer 1", 0.0)
//        Preferences.setDouble("odometer 2", 0.0)
//        Preferences.setDouble("odometer 3", 0.0)

        odometer0Entry.setDouble(Preferences.getDouble("odometer 0",5000.0))
        odometer1Entry.setDouble(Preferences.getDouble("odometer 1",5000.0))
        odometer2Entry.setDouble(Preferences.getDouble("odometer 2",5000.0))
        odometer3Entry.setDouble(Preferences.getDouble("odometer 3",5000.0))
        initializeSteeringMotors()
        println("prefs at enable=${Preferences.getDouble("odometer 0",0.0)}")
    }

    override fun postEnable() {
        brakeMode()
    }

    override fun onDisable() {
        coastMode()
        if (odometer0Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 0", odometer0Entry.getDouble(0.0))
        if (odometer1Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 1", odometer1Entry.getDouble(0.0))
        if (odometer2Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 2", odometer2Entry.getDouble(0.0))
        if (odometer3Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 3", odometer3Entry.getDouble(0.0))
    }

    override fun poseUpdate(poseTwist: SwerveDrive.Pose) {
        //MAPoseEstimator.addDriveData(Timer.getFPGATimestamp(), Twist2d(poseTwist.position.y, poseTwist.position.x, -poseTwist.heading.asRadians))
    }

    fun zeroGyro() {
        if (isBlueAlliance) {
            heading = 0.0.degrees
        } else {
            heading = 180.0.degrees
        }
        println("zeroed heading to $heading")//  alliance blue? ${AutoChooser.redSide}")
    }

    fun frontSpeakerResetOdom() {
        var resetPose = Vector2(48.2, 18.25)
        if (isBlueAlliance) {
            resetPose = resetPose.reflectAcrossField()
        }

        combinedPosition = resetPose.feet
        position = resetPose
        prevPosition = resetPose
        println("resetting to front speaker pos. $position")
    }

    fun convertTMMtoWPI(x:Length, y:Length, heading: Angle): Pose2d {
        val modX = -y.asMeters + fieldCenterOffsetInMeters.y
        val modY = x.asMeters + fieldCenterOffsetInMeters.x
        return Pose2d(modX,modY, Rotation2d((-heading+180.0.degrees).wrap().asRadians))
    }

    override suspend fun default() {

        periodic {
            var turn = 0.0
            if (OI.driveRotation.absoluteValue > 0.001) {
                turn = OI.driveRotation
            }

            val translation = OI.driveTranslation


            if (aimSpeaker || aimAmp) {
                val aimTurn = aimSpeakerAmpLogic()
                if (aimTurn != null) {
                    turn = aimTurn
                }
            }

            if (!useGyroEntry.exists()) {
                useGyroEntry.setBoolean(true)
            }
            val useGyro2 = useGyroEntry.getBoolean(true) && !DriverStation.isAutonomous()
            drive(
                translation * maxTranslation,
                turn * maxRotation,
                useGyro2,
                !(aimSpeaker || aimAmp)  // true for teleop, false when aiming
            )
        }
    }
    fun initializeSteeringMotors() {
        for (moduleCount in 0..3) { //changed to modules.indices, untested
            val module = (modules[moduleCount] as Module)
            module.turnMotor.setRawOffset(module.absoluteAngle.asDegrees)
            println("Module: $moduleCount analogAngle: ${module.absoluteAngle} motor: ${module.turnMotor.position}")
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
                return (digitalEncoder.absolutePosition.degrees * 360.0 * parameters.invertSteerFactor - angleOffset).wrap()
            }

        override val treadWear: Double
            get() = linearMap(0.0, 19000.0, 1.0, (1.0-kTread), odometer).coerceIn((1.0- kTread), 1.0)

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
            get() {
                return odometerEntry.getDouble(0.0)
            }
            set(value) {
                odometerEntry.setDouble(value)
            }

        override fun zeroEncoder() {
            driveMotor.position = 0.0
        }

        override var angleSetpoint: Angle = 0.0.degrees
            set(value) {
                field = value.unWrap(angle) * parameters.invertSteerFactor
                turnMotor.setPositionSetpoint(field.asDegrees)
            }

        override fun setDrivePower(power: Double) {
//            println("Drive power: ${power.round(6)}")
            driveMotor.setPercentOutput(power * parameters.invertDriveFactor)
        }


        val error: Angle
            get() = turnMotor.closedLoopError.degrees

        init {
            println("Drive.module.init")
            print(angle.asDegrees)
            driveMotor.config {
                brakeMode()
                //                    wheel diam / 12 in per foot * pi / gear ratio              * fudge factor
                feedbackCoefficient = 3.0 / 12.0 * Math.PI * (13.0/22.0 * 15.0/45.0 * 21.0/12.0) * (93.02 / 96.0)
                currentLimit(60, 68, 1)
                openLoopRamp(0.1)
            }
            turnMotor.config {
                feedbackCoefficient = (360.0 / 1.0 / 12.0 / 5.08) * (360.5 / 274.04)
                inverted(false)
                brakeMode()
                println("Absolute Angle: ${absoluteAngle.asDegrees}")
                setRawOffsetConfig(absoluteAngle.asDegrees)
                currentLimit(15, 20, 1)
                pid {
                    p(0.006)
//                    d(0.0000025)
                }
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
            val digitalAngle = (digitalEncoder.absolutePosition * 360.0 * parameters.invertSteerFactor).degrees.wrap().asDegrees
            angleOffset = digitalAngle.degrees
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


    fun aimSpeakerAmpLogic(): Double? {
        if (aimSpeaker || Robot.isAutonomous) {
            aimHeadingSetpoint = getAngleToSpeaker()
        } else {
            aimHeadingSetpoint = 90.0.degrees
        }

        val angleError = (heading - aimHeadingSetpoint).wrap()

        if (abs(angleError.asDegrees) > 2.0 || Robot.isAutonomous) {
            return aimPDController.update(angleError.asDegrees)
        }
        return null
    }

    fun getAngleToSpeaker(): Angle {
        val point = if (Pivot.pivotEncoderAngle > 90.0.degrees) ampPos else speakerPos
        val dVector = combinedPosition - point.feet
        return if (AprilTag.aprilTagsEnabled) kotlin.math.atan2(dVector.y.asFeet, dVector.x.asFeet).radians else if (isRedAlliance) 180.0.degrees + AprilTag.last2DSpeakerAngle.degrees else AprilTag.last2DSpeakerAngle.degrees
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
            power += 0.05
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            power -= 0.05
        }

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
    }
}

fun updatePos(driveStDevMeters: Double, vararg aprilPoses: GlobalPose) {
    val pos = combinedPosition
    prevCombinedPosition = pos
//                                            measurement, stdev
    val measurementsAndStDevs: MutableList<Pair<Vector2L, Double>> = mutableListOf()


    if (/*combinedPosition != Vector2L(0.0.inches, 0.0.inches) &&*/ DriverStation.isEnabled()) {

        testWheelPosition = combinedPosition + deltaPos //  + 0.5 * Drive.acceleration * dt * dt
        advantageWheelPoseEntry.setAdvantagePose(testWheelPosition, heading)

        measurementsAndStDevs.add(Pair(testWheelPosition, driveStDevMeters))
    }


    if (AprilTag.aprilTagsEnabled) {
        for (i in aprilPoses) {
            measurementsAndStDevs.add(Pair(i.pose, i.stDev))
        }
    }

    var a = Vector2L(0.0.inches, 0.0.inches)
    var b = 0.0

    for (i in measurementsAndStDevs) {
        a += i.first.asMeters.times(i.second.pow(-2)).meters
        b += i.second.pow(-2)
    }

    if (b != 0.0) {
        combinedPosition = a.asMeters.div(b).meters
    }

    prevCombinedPosition = pos
}

fun latencyAdjust(vector: Vector2L, latencySeconds: Double): Vector2L {
    val odomDiff = Drive.poseDiff(latencySeconds)
    return if (odomDiff != null) {
        vector + odomDiff.position.feet
    } else  {
        vector
    }
}

fun timeAdjust(vector: Vector2L, timestampSeconds: Double): Vector2L {
    return vector + position.feet - (Drive.lookupPose(timestampSeconds)?.position ?: position).feet
}