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
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.asFeet
import org.team2471.frc.lib.motion.following.poseDiff
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.units.Angle.Companion.tan
import org.team2471.frc2024.Drive.isRedAlliance
import kotlin.math.absoluteValue

object NoteDetector: Subsystem("NoteDetector") {

    private val pvtable = NetworkTableInstance.getDefault().getTable("photonvision")
    private val table = NetworkTableInstance.getDefault().getTable("NoteDetector")
    private val camera : PhotonCamera = PhotonCamera("notecam")
    private val noteAdvantagePosEntry = pvtable.getEntry("Advantage Note Pos")

    private val noteZeroPresentEntry = table.getEntry("NoteZeroPresent")
    private val noteOnePresentEntry = table.getEntry("NoteOnePresent")
    private val noteTwoPresentEntry = table.getEntry("NoteTwoPresent")
    private val noteThreePresentEntry = table.getEntry("NoteThreePresent")
    private val noteFourPresentEntry = table.getEntry("NoteFourPresent")

    private val noteHalfHeight = 1.0.inches
    private val camHeight = 9.796.inches
    private val camRobotCoords = Vector2(11.96.inches.asFeet, 0.0.inches.asFeet)
    private val cameraAngle = 15.0.degrees

    val middleNotesList: HashMap<Int, SchrodingerNote> = hashMapOf() // from closest to y=0 in wpi blue
    val closeNotesRedList : HashMap<Int, SchrodingerNote> = hashMapOf() // close notes n=0 is note closest to center field, n++ further away
    val closeNotesBlueList : HashMap<Int, SchrodingerNote> = hashMapOf()

    fun closeNote(n : Int) : Vector2 {
        if (isRedAlliance) {
            return closeNotesRedList[n]!!.position
        } else {
            return closeNotesBlueList[n]!!.position
        }
    }
    fun middleNote(n : Int) : Vector2 {
        return middleNotesList[n]!!.position
    }

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

    val closestIsMiddle: Boolean
        get() {
            val n = closestNote
            if (n != null) {
                println("Closest is middle: ${(26.135 - 2.5 < n.fieldCoords.x && n.fieldCoords.x < 26.135 + 2.5)} x: ${n.fieldCoords.x}")
                return (26.135 - 2.5 < n.fieldCoords.x && n.fieldCoords.x < 26.135 + 2.5) //middle of the field - offset 2.5 feet < x < middle of the field + offset 1.5 feet
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
                        if (n.fieldCoords.distance(Drive.combinedPosition.asFeet) > closest.fieldCoords.distance(Drive.combinedPosition.asFeet)) {
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


        for (n in 0 until 5 ) { //create five notes for the center of the field
            /*          -x
                 -- blue side --
                |               } blue amp
                |               |
            -y  | 0  1  2  3  4 |  +y
                |               |
                |               } red amp
                 --  red side --
                        +x
             */
            val offset: Vector2 = when (n) {
                0 -> Vector2(0.0, 0.0)
                1 -> Vector2(0.0, 0.0)
                2 -> Vector2(0.0, 0.0)
                3 -> Vector2(0.0, 0.0)
                4 -> Vector2(0.0, 0.0)
                else -> Vector2(0.0, 0.0)
            }
            val noteCoord = Vector2(27.135, (n * 66.0 + 29.64).inches.asFeet)
            println("creating note with ID: $n  Vector: $noteCoord  Offset: $offset  Offset Vector: ${noteCoord + offset}")
            middleNotesList[n] = SchrodingerNote(noteCoord + offset)
        }

        // Create close notes for red and blue
        for (n in 0 until 2) {
            closeNotesRedList[n] = SchrodingerNote(Vector2(114.0.inches.asFeet, 13.46 + (n * 57.0).inches.asFeet)) // Blue side notes, smaller is closer to center
            closeNotesBlueList[n] = SchrodingerNote(Vector2(54.27 - 114.0.inches.asFeet, 13.46 + (n * 57.0).inches.asFeet)) // Red Side notes, smaller is closer to center
        }


        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                val tempNotes : ArrayList<Note> = arrayListOf()
                notePosAdv = mutableListOf()

                if (camera.isConnected && camera.latestResult != null) {
                    for (target in camera.latestResult.targets) {
                        val robotCoords = getTargetRobotCoords(target)
                        val poseDiff = Drive.poseDiff(camera.latestResult.latencyMillis/1000)
                        var fieldCoords = robotCoordsToFieldCoords(robotCoords)
                        if (poseDiff != null) {
                            fieldCoords -= poseDiff.position
                        }
                        tempNotes.add(
                            Note(
                                robotCoords,
                                fieldCoords,
                                target.yaw,
                                Timer.getFPGATimestamp() - camera.latestResult.latencyMillis/1000
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
                if (notePosAdv.isNotEmpty()) {
                    noteAdvantagePosEntry.setDoubleArray(notePosAdv.first())
                } else {
                    noteAdvantagePosEntry.setDoubleArray(doubleArrayOf(0.0, 0.0, 0.0))
                }

                notes = tempNotes.toList()


                for (s in middleNotesList) {
                    s.value.isPresent = false
                    for (n in notes) {
                        if ((n.fieldCoords.x - s.value.position.x).absoluteValue < 6.0.inches.asFeet && (n.fieldCoords.y - s.value.position.y).absoluteValue < 6.0.inches.asFeet) {
                            s.value.isPresent = true
                        }
                    }
                }
                middleNotesList[0]?.let { noteZeroPresentEntry.setBoolean(it.isPresent) }
                middleNotesList[1]?.let { noteOnePresentEntry.setBoolean(it.isPresent) }
                middleNotesList[2]?.let { noteTwoPresentEntry.setBoolean(it.isPresent) }
                middleNotesList[3]?.let { noteThreePresentEntry.setBoolean(it.isPresent) }
                middleNotesList[4]?.let { noteFourPresentEntry.setBoolean(it.isPresent) }
                SmartDashboard.putBoolean("NoteCameraIsConnected", camera.isConnected)
            }
        }
    }
    fun getTargetRobotCoords(target : PhotonTrackedTarget): Vector2 {
        val distance = distanceCurve.getValue(target.pitch).feet
        val xOffset = distance * -tan(target.yaw.degrees)
        return Vector2(distance.asFeet, xOffset.asFeet) + camRobotCoords
    }

    fun robotCoordsToFieldCoords(robotCoords : Vector2): Vector2 {
        return robotCoords.rotateDegrees(Drive.heading.asDegrees) + Drive.combinedPosition.asFeet
    }

    fun closestNoteAtPosition(expectedPos : Vector2, maximumErr: Double = 3.5) : Boolean {
        var note = closestNote
        if (note != null) {
            if ((expectedPos - note.fieldCoords).length < maximumErr) { // is it a different note
                return true
            }
        }
        return false
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