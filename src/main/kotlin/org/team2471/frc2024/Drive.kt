package org.team2471.frc2024

import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.control.PDConstantFController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.*
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.motion_profiling.following.SwerveParameters
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.isReal
import org.team2471.frc2024.Drive.position
import org.team2471.frc2024.gyro.Gyro
import kotlin.math.*

@OptIn(DelicateCoroutinesApi::class)
object Drive : Subsystem("Drive"), SwerveDrive {
    val robotHalfWidth = (25.0/2.0).inches
    val table = NetworkTableInstance.getDefault().getTable(name)

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

    val aimHeadingSetpointEntry = table.getEntry("aimHeadingSetpoint")

    val totalDriveCurretEntry = table.getEntry("Total Drive Current")
    val totalTurnCurrentEntry = table.getEntry("Total Turn Current")

    val velocityEntry = table.getEntry("Velocity")
    val angularVelocityEntry = table.getEntry("Angular Velocity")

    val useGyroEntry = table.getEntry("Use Gyro")
    val gyroIsConnectedEntry = table.getEntry("Gyro Connected")

    val plannedPathEntry = table.getEntry("Planned Path")
    val actualRouteEntry = table.getEntry("Actual Route")
    val distanceEntry = table.getEntry("Distance From Speaker Drive")

    private val advantagePoseEntry = table.getEntry("Drive Advantage Pose")
    private val predictedAdvantagePoseEntry = table.getEntry("Predicted Advantage Pose")

    private val deltaPosEntry = table.getEntry("DeltaPos")
    private val deltaPosSecEntry = table.getEntry("DeltaPos 2")

    val advantageWheelPoseEntry = table.getEntry("Test Wheel Advantage Pose")

    private val advantageCombinedPoseEntry = table.getEntry("Combined Advantage Pose")

    private val aimTargetEntry = table.getEntry("Aim Target")

    override val parameters: SwerveParameters = SwerveParameters(
        gyroRateCorrection = 0.0,
        kpPosition = 0.32,
        kdPosition = 0.6,
        kPositionFeedForward = 0.05,
        kpHeading = 0.005,
        kdHeading = 0.02,
        kHeadingFeedForward = 0.001,
        kMoveWhileSpin = if (isReal) 27.0 else 59.0,
    )


    /**
     * Coordinates of modules
     * **/
    override val modules: Array<SwerveDrive.Module> = arrayOf(
        Module(
            MotorController(FalconID(Falcons.FRONT_LEFT_DRIVE, "Drive/FLD")),
            MotorController(SparkMaxID(Sparks.FRONT_LEFT_STEER, "Drive/FLS")),
            Vector2(-10.75, 10.75).inches,
            Preferences.getDouble("Angle Offset 0",if (Robot.isCompBot) 26.69 else 81.876).degrees,
            DigitalSensors.FRONT_LEFT,
            odometer0Entry,
            0
        ),
        Module(
            MotorController(FalconID(Falcons.FRONT_RIGHT_DRIVE, "Drive/FRD")),
            MotorController(SparkMaxID(Sparks.FRONT_RIGHT_STEER, "Drive/FRS")),
            Vector2(10.75, 10.75).inches,
            Preferences.getDouble("Angle Offset 1",if (Robot.isCompBot) -19.67 else -35.897).degrees,
            DigitalSensors.FRONT_RIGHT,
            odometer1Entry,
            1
        ),
        Module(
            MotorController(FalconID(Falcons.BACK_RIGHT_DRIVE, "Drive/BRD")),
            MotorController(SparkMaxID(Sparks.BACK_RIGHT_STEER, "Drive/BRS")),
            Vector2(10.75, -10.75).inches,
            Preferences.getDouble("Angle Offset 2",if (Robot.isCompBot) -35.17 else -150.539).degrees,
            DigitalSensors.BACK_RIGHT,
            odometer2Entry,
            2
        ),
        Module(
            MotorController(FalconID(Falcons.BACK_LEFT_DRIVE, "Drive/BLD")),
            MotorController(SparkMaxID(Sparks.BACK_LEFT_STEER, "Drive/BLS")),
            Vector2(-10.75, -10.75).inches,
            Preferences.getDouble("Angle Offset 3",if (Robot.isCompBot) -159.03 else -104.767).degrees,
            DigitalSensors.BACK_LEFT,
            odometer3Entry,
            3
        )
    )

    private val gyro = Gyro
    private var gyroOffset = 0.0.degrees

    override var heading: Angle
        get() = (gyroOffset - gyro.angle).wrap()
        set(value) {
//            gyro.reset()
            gyroOffset = gyro.angle + value
        }
    override val gyroConnected: Boolean get() = Gyro.isConnected

    override val headingRate: AngularVelocity
        get() = -gyro.rate.degrees.perSecond

    override var velocity = Vector2(0.0, 0.0)
    override var acceleration: Vector2 = Vector2(0.0, 0.0)
    override var position = Vector2(0.0, 0.0)
        set(value) {
            field = value
            lastResetTime = Timer.getFPGATimestamp()
        }

    override var deltaPos = Vector2L(0.0.inches, 0.0.inches)

    var testWheelPosition: Vector2L = position.feet
//                   feet seconds fps fudge
    val driveStDevM = (2.5 / 30.0 / 50.0 * 15.0).feet.asMeters

    override var lastResetTime: Double = 0.0


    override var robotPivot = Vector2(0.0, 0.0).inches
    override var headingSetpoint = 0.0.degrees

    override val carpetFlow = Vector2(1.0, 0.0) //dcmp comp 1.0  practice -1.0
    override val kCarpet = 0.0 // 0.025 // how much downstream and upstream carpet directions affect the distance, for no effect, use  0.0 (2.12% more distance downstream)
    override val kTread = 0.0 // 0.04 // how much of an effect treadWear has on distance (fully worn tread goes 4% less than full tread)  0.0 for no effect
    override val plannedPath: NetworkTableEntry = plannedPathEntry
    override val actualRoute: NetworkTableEntry = actualRouteEntry

    val autoPDController = PDConstantFController(0.015, 0.04, 0.02)
    val teleopPDController =  PDConstantFController( 0.01,0.075, 0.025)
//    val teleopPDController =  PDConstantFController(parameters.kpHeading, parameters.kdHeading, parameters.kHeadingFeedForward)

    var aimPDController = teleopPDController

//    var aimSpeaker = false
//    var aimNote = false
//    var aimAmp = false

    var aimTarget: AimTarget = AimTarget.NONE

    val speakerPos
        get() = if (isRedAlliance) Vector2(652.76.inches.asFeet - 7.0.inches.asFeet, 218.5.inches.asFeet) else Vector2(-1.575.inches.asFeet + 7.0.inches.asFeet, 218.5.inches.asFeet) //orig 218.5 for both -- aiming left   3/28 642.73.inches.asFeet

    val offsetSpeakerPose
        get() = speakerPos + Vector2(
            0.0,
            linearMap(
                218.42.inches.asFeet - 13.0,
                218.42.inches.asFeet + 13.0,
                12.0.inches.asFeet, -12.0.inches.asFeet,
                AprilTag.position.y.asFeet)
        )

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
            val headingRadEntry = table.getEntry("Heading Rad")
            val xEntry = table.getEntry("Position X")
            val yEntry = table.getEntry("Position Y")

            if (!SmartDashboard.containsKey("DemoSpeed")) {
                println("DemoSpeed does not exist, setting it to 1.0")
                SmartDashboard.getEntry("DemoSpeed").setDouble(1.0)
                SmartDashboard.setPersistent("DemoSpeed")
            }

            val encoderCounterList = arrayOf(0, 0, 0, 0)
            val previousAngles = arrayOf(0.0.degrees, 0.0.degrees, 0.0.degrees, 0.0.degrees)

            val wheelCenterOffsets = arrayOfNulls<Pose3d>(modules.indices.count())
            val returningTranslations = arrayOfNulls<Translation3d>(modules.indices.count())

            for (i in modules.indices) {
                val m = modules[i] as Module
                val modelAdjustedAngle = m.modulePosition.rotateDegrees(90.0 * (i + 1.0)).asMeters
                val modelOffset = Translation3d(modelAdjustedAngle.x, (-1.5).inches.asMeters, modelAdjustedAngle.y)
                val wheelCenterOffset = Pose3d(
                    Translation3d(-0.058, 0.0051, 0.017),
                    Rotation3d(90.0.degrees.asRadians, 0.0.degrees.asRadians, 90.0.degrees.asRadians)
                ).transformBy(Transform3d(modelOffset, Rotation3d()))
                val returningTranslation = Translation3d(-modelOffset.x, -modelOffset.z, modelOffset.y)
                    .rotateBy(Rotation3d(0.0, 0.0, (-90.0.degrees * (i.toDouble() + 2.0)).asRadians))
                wheelCenterOffsets[i] = wheelCenterOffset
                returningTranslations[i] = returningTranslation
            }

            println("in init just before periodic")
            periodic {
                recordOdometry()

                val batteryVolt = RobotController.getBatteryVoltage() > 12.7
                SmartDashboard.putBoolean("Battery Good", batteryVolt)
                val demoDisabled = demoSpeed == 1.0
                SmartDashboard.putBoolean("FullSpeed", demoDisabled)

                val (x, y) = position
                xEntry.setDouble(x)
                yEntry.setDouble(y)
                headingEntry.setDouble(heading.asDegrees)
                headingRadEntry.setDouble(heading.asRadians)

                advantagePoseEntry.setDoubleArray(
                    doubleArrayOf(
                        position.x.feet.asMeters,
                        position.y.feet.asMeters,
                        heading.asDegrees
                    )
                )

//                Logger.recordOutput("deltaPos", deltaPos.length.asFeet)
//                Logger.recordOutput("velocity", velocity.length)
//                Logger.recordOutput("acceleration", acceleration.length)

                deltaPosEntry.setAdvantagePose(deltaPos, heading)
                deltaPosSecEntry.setAdvantagePose((deltaPos * (0.5 / 50)), heading)

                val aimTargetPoint = if (aimTarget == AimTarget.SPEAKER) arrayOf(offsetSpeakerPose.feet)else(arrayOf())
                aimTargetEntry.setAdvantagePoses(aimTargetPoint)

                advantageCombinedPoseEntry.setAdvantagePose(AprilTag.position, heading)

                motorAngle0Entry.setDouble((modules[0] as Module).angle.wrap().asDegrees)
                motorAngle1Entry.setDouble((modules[1] as Module).angle.wrap().asDegrees)
                motorAngle2Entry.setDouble((modules[2] as Module).angle.wrap().asDegrees)
                motorAngle3Entry.setDouble((modules[3] as Module).angle.wrap().asDegrees)

                absoluteAngle0Entry.setDouble((modules[0] as Module).absoluteAngle.asDegrees)
                absoluteAngle1Entry.setDouble((modules[1] as Module).absoluteAngle.asDegrees)
                absoluteAngle2Entry.setDouble((modules[2] as Module).absoluteAngle.asDegrees)
                absoluteAngle3Entry.setDouble((modules[3] as Module).absoluteAngle.asDegrees)

                velocityEntry.setDouble(velocity.length)
                angularVelocityEntry.setDouble(headingRate.changePerSecond.asDegrees)

                val setpointStates = arrayOfNulls<SwerveModuleState>(4)
                val absoluteStates = arrayOfNulls<SwerveModuleState>(4)
                val motorAngleStates = arrayOfNulls<SwerveModuleState>(4)
                var totalDriveCurrent = 0.0
                var totalTurnCurrent = 0.0

                for (i in modules.indices) {
                    val m = modules[i] as Module
                    val absoluteAngle = m.absoluteAngle
                    val angle = m.angle
                    val absoluteSpeed = (m.speed / 25.0).coerceIn(-1.0, 1.0)
                    val angleSetpoint = m.angleSetpoint
                    val speedSetpoint = m.power

                    totalDriveCurrent += m.driveCurrent
                    totalTurnCurrent += m.turnMotor.current

                    if (Robot.isDisabled) {
                        if (absoluteAngle == previousAngles[i]) {
                            encoderCounterList[i]++
                        } else {
                            m.encoderConnected = true
                            encoderCounterList[i] = 0
                        }
                        if (encoderCounterList[i] >= 40) {
                            m.encoderConnected = false
                        }
                        previousAngles[i] = absoluteAngle
                    }
                    setpointStates[i] = SwerveModuleState(speedSetpoint, angleSetpoint.asRotation2d)
                    absoluteStates[i] = SwerveModuleState(absoluteSpeed, absoluteAngle.asRotation2d)
                    motorAngleStates[i] = SwerveModuleState(absoluteSpeed, angle.asRotation2d)

                    //advantageScope 3d component visualization
                    val wheelCenterOffset = wheelCenterOffsets[i]
                    val returningTranslation = returningTranslations[i]
                    if (wheelCenterOffset != null && returningTranslation != null) {
                        val rotatedModule = wheelCenterOffset
                            .rotateBy(Rotation3d(0.0, (m.currDistance * 12.0 / 3.0 / Math.PI).rotations.asRadians, -m.angle.asRadians))
                        val newCalculatedPose = Pose3d(rotatedModule.translation - returningTranslation, rotatedModule.rotation)

                        Robot.logComponent("Components/${i + 3} Module$i", newCalculatedPose)
                    }
                }
                try {
                    Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
                    Logger.recordOutput("SwerveStates/AbsoluteAngles", *absoluteStates)
                    Logger.recordOutput("SwerveStates/MotorAngles", *motorAngleStates)
                    Logger.recordOutput("Drive/Heading", -heading.asRotation2d)
                    Logger.recordOutput("Drive/Position", position.toPose2d(heading.asDegrees))
                } catch (_: Exception) {}
                totalDriveCurretEntry.setDouble(totalDriveCurrent)
                totalTurnCurrentEntry.setDouble(totalTurnCurrent)

                gyroIsConnectedEntry.setBoolean(gyro.isConnected)

                if (!Robot.inComp) {
                    turnMotor0CurrentEntry.setDouble((modules[0] as Module).turnMotor.current)
                    turnMotor1CurrentEntry.setDouble((modules[1] as Module).turnMotor.current)
                    turnMotor2CurrentEntry.setDouble((modules[2] as Module).turnMotor.current)
                    turnMotor3CurrentEntry.setDouble((modules[3] as Module).turnMotor.current)

                    driveMotor0CurrentEntry.setDouble((modules[0] as Module).driveCurrent)
                    driveMotor1CurrentEntry.setDouble((modules[1] as Module).driveCurrent)
                    driveMotor2CurrentEntry.setDouble((modules[2] as Module).driveCurrent)
                    driveMotor3CurrentEntry.setDouble((modules[3] as Module).driveCurrent)

                    drivePowerEntry.setDouble((modules[0] as Module).power)
                }

                aimHeadingSetpointEntry.setDouble(aimHeadingSetpoint.asDegrees)

                distanceEntry.setDouble(distanceFromSpeakerDrivePos)

                if (Robot.isAutonomous && aimTarget == AimTarget.SPEAKER) {
                    var turn = 0.0
                    val aimTurn = aimSpeakerAmpLogic()

                    if (aimTurn != null) {
                        turn = aimTurn
                    }

                    if (!useGyroEntry.exists()) {
                        useGyroEntry.setBoolean(true)
                        useGyroEntry.setPersistent()
                    }
                    val useGyro2 = useGyroEntry.getBoolean(true) && !DriverStation.isAutonomous()

                    drive(
                        OI.driveTranslation * maxTranslation,
                        turn * maxRotation,
                        useGyro2,
                        aimTarget == AimTarget.SPEAKER  // true for teleop, false when aiming
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
        initializeSteeringMotors()
        println("prefs at enable=${Preferences.getDouble("odometer 0",0.0)}")
    }

    override fun postEnable() {
        GlobalScope.launch {
            println("INSIDE DRIVE POST ENABLE!!!!!!!!!!!! ${Robot.totalTimeTaken()}")
            odometer0Entry.setDouble(Preferences.getDouble("odometer 0",5000.0))
            odometer1Entry.setDouble(Preferences.getDouble("odometer 1",5000.0))
            odometer2Entry.setDouble(Preferences.getDouble("odometer 2",5000.0))
            odometer3Entry.setDouble(Preferences.getDouble("odometer 3",5000.0))
            brakeMode()
            println("after break mode in drive postEnable ${Robot.totalTimeTaken()}")
        }
    }

    override fun onDisable() {
        coastMode()
        if (odometer0Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 0", odometer0Entry.getDouble(0.0))
        if (odometer1Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 1", odometer1Entry.getDouble(0.0))
        if (odometer2Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 2", odometer2Entry.getDouble(0.0))
        if (odometer3Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 3", odometer3Entry.getDouble(0.0))
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

        AprilTag.position = resetPose.feet
        position = resetPose
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


            if (aimTarget != AimTarget.NONE) {
//                if (demoMode) {
//                    if ( aimTarget == AimTarget.DEMOTAG) {
//                        turn = aimPDController.update(Limelight.limelight.tx.asDegrees) / maxRotation
//                    }
//                } else {
                    val aimTurn = aimSpeakerAmpLogic()
                    if (aimTurn != null) {
                        turn = aimTurn
                    }
//                }
            }

            if (!useGyroEntry.exists()) {
                useGyroEntry.setBoolean(true)
                useGyroEntry.setPersistent()
            }
            val useGyro2 = useGyroEntry.getBoolean(true) && !DriverStation.isAutonomous()
            drive(
                translation * maxTranslation,
                turn * maxRotation,
                useGyro2,
                aimTarget == AimTarget.NONE  // true for teleop, false when aiming
            )
        }
    }

    suspend fun dynamicDriveBetweenPoints(startHeading: Angle, endHeading: Angle, vararg points: Vector2, setVelocity: Double = 10.0, earlyExit: (Double) -> Boolean = {false}) {
        val newPath = Path2D("newPath")

        for (p in points) {
            newPath.addVector2(p)
        }
        val duration = newPath.length / setVelocity
        newPath.easeCurve.setMarkBeginOrEndKeysToZeroSlope(false)  // if this doesn't work, we could add with tangent manually
        newPath.addEasePoint(0.0, 0.0)
        newPath.addEasePoint(duration, 1.0)
        newPath.addHeadingPoint(0.0, startHeading.asDegrees)
        newPath.addHeadingPoint(duration, endHeading.unWrap(startHeading).asDegrees)
        Drive.driveAlongPath(newPath, false, earlyExit = earlyExit)
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
        override val modulePosition: Vector2L,
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
            get() = turnMotor.position.degrees

        var encoderConnected: Boolean = true

        val digitalEncoder : DutyCycleEncoder = DutyCycleEncoder(digitalInputID)

        val absoluteAngle: Angle
            get() = /*if (encoderConnected) {*/
                    (digitalEncoder.absolutePosition.degrees * 360.0 - angleOffset).wrap()
//                } else {
//                    //encoder not connected, assume wheel is zeroed
//                    (0.0.degrees - angleOffset).wrap()
//                }

        override val treadWear: Double
            get() = linearMap(0.0, 19000.0, 1.0, (1.0-kTread), odometer).coerceIn((1.0- kTread), 1.0)

        val driveCurrent: Double
            get() = driveMotor.current

        private val pdController = PDController(P, D)

        override val speed: Double
            get() = driveMotor.velocity

        override val acceleration: Double
            get() = driveMotor.acceleration

        val power: Double
            get() {
                return driveMotor.output
            }

        override val currDistance: Double
            get() = driveMotor.position

        override var prevDistance: Double = 0.0
        override var prevAngle: Angle = angle

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
                field = value.unWrap(angle)
                turnMotor.setPositionSetpoint(field.asDegrees)
            }

        override fun setDrivePower(power: Double) {
//            println("Drive power: ${power.round(6)}")
            driveMotor.setPercentOutput(power)
        }


        val error: Angle
            get() = turnMotor.closedLoopError.degrees

        init {
            println("Drive.module.init")
            print(angle.asDegrees)
            driveMotor.config {
                brakeMode()
                //                    wheel diam / 12 in per foot * pi / gear ratio                                               * fudge
            feedbackCoefficient = 3.0 / 12.0 * Math.PI * ((if (Robot.isCompBot) 12.0 else 13.0)/22.0 * 15.0/45.0 * 21.0/12.0)// * (17.0 / 45.0) //* (93.02 / 96.0) * 1.04
                currentLimit(55, 60, 1)
                openLoopRamp(0.1)
                configSim(DCMotor.getKrakenX60Foc(1), 0.005)
            }
            turnMotor.config {
                feedbackCoefficient = (360.0 / 1.0 / 12.0 / 5.08) * (360.5 / 274.04)
                inverted(true)
                brakeMode()
                println("Absolute Angle: ${absoluteAngle.asDegrees}")
                setRawOffsetConfig(absoluteAngle.asDegrees)
                currentLimit(15, 20, 1)
                pid {
                    p(6.144, 2.33)
//                    d(0.0000025 * 1024.0)
                }
                configSim(DCMotor.getNeo550(1), 0.0000065)
            }
//            GlobalScope.launch {
//                periodic {
////                    println("${turnMotor.motorID}   ${ round(absoluteAngle.asDegrees, 2) }")
//
//                }
//            }
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
            val digitalAngle = (digitalEncoder.absolutePosition * 360.0).degrees.wrap().asDegrees
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

    val velocityPositionController = PDController(0.1, 0.0)
    val velocityHeadingController = PDController(0.001, 0.0)

    fun driveWithVelocity(wantedVelocity: Vector2, turnVelocity: Angle) {
        val x = wantedVelocity.x
        val y = wantedVelocity.y
        val translation = Vector2(velocityPositionController.update(x - velocity.x), velocityPositionController.update(y - velocity.y))

        val turn = velocityHeadingController.update(turnVelocity.asDegrees - headingRate.changePerSecond.asDegrees)

        println(turn)

        drive(translation, turn, false)
    }

    fun aimSpeakerAmpLogic(smoothing: Boolean = false): Double? {
        aimHeadingSetpoint = when(aimTarget) {
            AimTarget.SPEAKER -> getAngleToSpeaker(true) + if (DriverStation.isAutonomous()) 4.0.degrees else 2.7.degrees
            AimTarget.AMP -> 90.0.degrees
            AimTarget.GAMEPIECE -> -NoteDetector.angleToClosestNote()!!
            AimTarget.PODIUM -> if (isRedAlliance) 209.0.degrees else -27.0.degrees  //podium aiming
            AimTarget.PASS -> if (isRedAlliance) 220.0.degrees else -28.0.degrees  //pass aiming
            else -> getAngleToSpeaker()
        }

        if (smoothing) {
            aimHeadingSetpoint = interpTo(heading.asDegrees, aimHeadingSetpoint.asDegrees, 20.0).degrees
        }
        val angleError = (heading - aimHeadingSetpoint).wrap()

        if (abs(angleError.asDegrees) > 2.0 || Robot.isAutonomous) {
            return aimPDController.update(angleError.asDegrees) / maxRotation //dividing by maxRotation for demo mode to have no effect
        }
        return null
    }

    fun aim(): Double? {
        if (AprilTag.position.x < 29.0.feet && AprilTag.position.x > 25.0.feet) {
            val dVector = AprilTag.position - Vector2L(if (isBlueAlliance) 25.0.feet else 29.0.feet, 27.0.feet)
            aimHeadingSetpoint = atan2(dVector.y.asFeet, dVector.x.asFeet).radians
        } else if (isRedAlliance) {
            aimHeadingSetpoint = 180.0.degrees
        } else {
            aimHeadingSetpoint = 0.0.degrees
        }

        val angleError = (heading - aimHeadingSetpoint).wrap()

        if (abs(angleError.asDegrees) > 2.0 || Robot.isAutonomous) {
            return aimPDController.update(angleError.asDegrees)
        }
        return null
    }

    fun getAngleToSpeaker(useSpinCompensation: Boolean = false): Angle {
        val point = if (useSpinCompensation) {
                offsetSpeakerPose
            } else {
                speakerPos
            }

        val dVector = AprilTag.position - point.feet
        return atan2(dVector.y.asFeet, dVector.x.asFeet).radians
    }
}

fun Drive.abortPath(): Boolean {
    return isHumanDriving
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


enum class AimTarget {
    SPEAKER,
    AMP,
    GAMEPIECE,
    PODIUM,
    PASS,
    DEMOTAG,
    NONE
}


//fun GlobalPose.latencyAdjust() {
//    this.pose += position.feet - (Drive.lookupPose(this.timestampSeconds)?.position ?: position).feet
//}