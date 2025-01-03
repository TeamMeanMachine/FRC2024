package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.asFeet
import org.team2471.frc.lib.motion.following.poseDiff
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.units.Angle.Companion.tan
import org.team2471.frc2024.Drive.isRedAlliance
import kotlin.math.absoluteValue

object NoteDetector {

    private val pvtable = NetworkTableInstance.getDefault().getTable("photonvision")
    private val table = NetworkTableInstance.getDefault().getTable("NoteDetector")
    val camera : PhotonCamera = PhotonCamera("notecam")
    private val noteAdvantagePos0Entry = table.getEntry("Advantage Note Pos0")
    private val noteAdvantagePos1Entry = table.getEntry("Advantage Note Pos1")
    private val noteAdvantagePos2Entry = table.getEntry("Advantage Note Pos2")
    private val noteAdvantagePos3Entry = table.getEntry("Advantage Note Pos3")

    val numberOfNotesEntry = table.getEntry("Number of Notes")

    val camRobotCoords = Vector2(11.96.inches.asFeet, 0.0.inches.asFeet)

    val middleNotesSpoilerYaw = MotionCurve()
    val middleNotesSpoilerPitch = MotionCurve()
    val middleNotesSpoilerRPM = MotionCurve()

    val seesNote: Boolean
        get() {
            return if (camera.isConnected) {
                try {
                    camera.latestResult.hasTargets()
                } catch (e: Exception) {
                    println("Could not get latest result from note camera")
                    false
                }
            } else {
                false
            }
        }
    var seesNoteCounter = 0

    val closestIsValid: Boolean
        get() {
            val n = closestNote
            if (n != null) {
                if (isRedAlliance) {
                    return n.fieldCoords.x > 26.135 - 1.5 //if x > middle of the field - offset 1.5 feet
                } else {
                    return n.fieldCoords.x < 26.135 + 1.5 //if x < middle of the field + offset 1.5 feet
                }
            }
            return false
        }

    private val distanceCurve = MotionCurve()
    var notes : List<Note> = listOf()

    val closestNote: Note?
        get() {
            if (notes.isNotEmpty()) {
                var closest: Note? = null
                for (n in notes) {
                    if (closest != null) {
                        if (n.fieldCoords.distance(AprilTag.position.asFeet) > closest.fieldCoords.distance(AprilTag.position.asFeet)) {
                            closest = n
                        }
                    } else {
                        closest = n
                    }
                }
                return closest
            } else {
                return null
            }
        }

    var notePosAdv: MutableList<Array<Double>> = mutableListOf()

    init {
        println("init note detector")
        PhotonCamera.setVersionCheckEnabled(false)

        distanceCurve.setMarkBeginOrEndKeysToZeroSlope(false) // IDK IF THIS GOES BFORE OR AFTER
        distanceCurve.storeValue(-14.7, 14.0.inches.asFeet)
        distanceCurve.storeValue(-11.3, 1.5)
        distanceCurve.storeValue(-6.14, 2.0)
        distanceCurve.storeValue(-2.3, 2.5)
        distanceCurve.storeValue(0.35, 3.0)
        distanceCurve.storeValue(1.8, 3.5)
        distanceCurve.storeValue(3.5, 4.0)
        distanceCurve.storeValue(6.85, 6.0)
        distanceCurve.storeValue(8.49, 8.0)
        distanceCurve.storeValue(9.7, 10.0)
        distanceCurve.storeValue(10.6, 12.0)
        distanceCurve.setMarkBeginOrEndKeysToZeroSlope(false)


        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                val tempNotes : ArrayList<Note> = arrayListOf()
                notePosAdv = mutableListOf()

                val latestResult = camera.latestResult
                if (camera.isConnected && latestResult != null && latestResult.hasTargets()) {
                    for (target in latestResult.targets) {
                        val robotCoords = getTargetRobotCoords(target)
                        val poseDiff = Drive.poseDiff(latestResult.timestampSeconds)
                        var fieldCoords = robotCoordsToFieldCoords(robotCoords)
                        if (poseDiff != null) {
                            fieldCoords -= poseDiff.position
                        }
                        tempNotes.add(
                            Note(
                                robotCoords,
                                fieldCoords,
                                target.yaw,
                                latestResult.timestampSeconds
                            )
                        )
//                        println(("pitch: ${target.pitch}"))
//                        println("x: ${robotCoords.x}\ny:${robotCoords.y}")

                        //advantage scope list
                        notePosAdv.add(
                            arrayOf(
                                fieldCoords.x.feet.asMeters, //x
                                fieldCoords.y.feet.asMeters, //y
                                0.0 //z
                            )
                        )
                    }
                }
                val emptyArray = doubleArrayOf(0.0, 0.0, 0.0)
                try {noteAdvantagePos0Entry.setDoubleArray(notePosAdv[0])} catch(_:Exception) {noteAdvantagePos0Entry.setDoubleArray(emptyArray)}
                try {noteAdvantagePos1Entry.setDoubleArray(notePosAdv[1])} catch(_:Exception) {noteAdvantagePos1Entry.setDoubleArray(emptyArray)}
                try {noteAdvantagePos2Entry.setDoubleArray(notePosAdv[2])} catch(_:Exception) {noteAdvantagePos2Entry.setDoubleArray(emptyArray)}
                try {noteAdvantagePos3Entry.setDoubleArray(notePosAdv[3])} catch(_:Exception) {noteAdvantagePos3Entry.setDoubleArray(emptyArray)}

                notes = tempNotes.toList()


                SmartDashboard.putBoolean("NoteCameraIsConnected", camera.isConnected)

                numberOfNotesEntry.setDouble(notes.size.toDouble())

            }
        }
    }
    fun getTargetRobotCoords(target : PhotonTrackedTarget): Vector2 {
        val distance = distanceCurve.getValue(target.pitch).feet
        val xOffset = distance * -tan(target.yaw.degrees)
        return Vector2(distance.asFeet, xOffset.asFeet) + camRobotCoords
    }

    fun robotCoordsToFieldCoords(robotCoords : Vector2): Vector2 {
        return robotCoords.rotateDegrees(Drive.heading.asDegrees) + AprilTag.position.asFeet
    }

    fun closestNoteIsAtPosition(expectedPos : Vector2, maximumErr: Double = 3.5) : Boolean {
        var note = closestNote

        if (seesNote) {
            seesNoteCounter ++
        } else {
            seesNoteCounter = 0
        }

        if (note != null) {
            println("Expected: $expectedPos   Note: ${note.fieldCoords}   Length: ${(expectedPos - note.fieldCoords).length}")

            if ((expectedPos - note.fieldCoords).length < maximumErr) { // is it a different note
                return true
            } else {
                println("Note: ${note.fieldCoords}  Pos: $expectedPos")
                return false
            }
        }
        return seesNoteCounter > 1

//        return false
    }

    fun getNearNoteAtPosition(expectedPos : Vector2, maximumErr: Double = 3.5) : Note? {
        if (notes.isNotEmpty()) {
            for (note in notes) {
                val hi = note.fieldCoords.distance(expectedPos).absoluteValue
                println("Expected: $expectedPos   Note: ${note.fieldCoords}   Distance from expected: $hi")
                if (hi < maximumErr) {
                    println("less than maxError")
                    return note
                }
            }
        }
        return null
    }

    fun closestIsMiddleAdjust(maximumErr: Double = 2.5) : Boolean {
        val n = closestNote
        if (n != null) {
//            println("Closest is middle: ${(26.135 - maximumErr < n.fieldCoords.x && n.fieldCoords.x < 26.135 + maximumErr)} x: ${n.fieldCoords.x}")
            return (26.135 - maximumErr < n.fieldCoords.x && n.fieldCoords.x < 26.135 + maximumErr) //middle of the field - offset 2.5 feet < x < middle of the field + offset 1.5 feet
        }
        return false
    }

    fun angleToClosestNote() : Angle? {
        if (notes.isEmpty()) return null
//        var poseDiff =  Drive.poseDiff(Timer.getFPGATimestamp() - notes[0].timestampSeconds)
//        if (poseDiff == null) poseDiff = SwerveDrive.Pose(Vector2(0.0, 0.0), 0.0.degrees)
        return if (closestNote != null) closestNote?.robotCoords?.angle else null
    }

}

data class Note(
    val robotCoords: Vector2,
    val fieldCoords: Vector2,
    val yawOffset : Double,
    val timestampSeconds: Double
)
data class SchrodingerNote (
    val position: Vector2,
    var isPresent: Boolean = false
)