package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.units.Angle.Companion.tan
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
    private val camRobotCoords = Vector2(0.0.inches.asFeet, 11.96.inches.asFeet)
    private val cameraAngle = -15.0.degrees

    val noteList: HashMap<Int, SchrodingerNote> = hashMapOf()

    val seesNote: Boolean
        get() {
            return if (camera.isConnected) {
                camera.latestResult.hasTargets()
            } else {
                false
            }
        }

    private val distanceCurve = MotionCurve()
    var notes : List<Note> = listOf()

    val closestNote: Note?
        get() {
            if (notes.isNotEmpty()) {
                var closest: Note? = null
                for (n in notes) {
                    if (closest != null) {
                        if (n.fieldCords.distance(Drive.combinedPosition) > closest.fieldCords.distance(Drive.combinedPosition)) {
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
        distanceCurve.storeValue(-1.6, 109.0)
        distanceCurve.storeValue(-17.5, 12.0)
        distanceCurve.storeValue(-6.48, 46.0)
        distanceCurve.storeValue(-3.5, 73.0)

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
            val noteCord = Vector2(27.216, (n * 66.0 + 29.64).inches.asFeet)
            println("creating note with ID: $n  Vector: $noteCord  Offset: $offset  Offset Vector: ${noteCord + offset}")
            noteList[n] = SchrodingerNote(noteCord + offset)
        }

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                val tempNotes : ArrayList<Note> = arrayListOf()
                notePosAdv = mutableListOf()

                if (camera.isConnected) {
                    for (target in camera.latestResult.targets) {
                        val robotCords = getTargetRobotCords(target)
                        val fieldCords = robotCordsToFieldCords(robotCords)
                        tempNotes.add(
                            Note(
                                robotCords,
                                fieldCords,
                                camera.latestResult.timestampSeconds
                            )
                        )

                        //advantage scope list
                        notePosAdv.add(
                            arrayOf(
                                fieldCords.x.feet.asMeters, //x
                                fieldCords.y.feet.asMeters, //y
                                0.0 //z
                            )
                        )
                    }
                }
                if (notePosAdv.isNotEmpty()) {
                    noteAdvantagePosEntry.setDoubleArray(notePosAdv.first())
                }
                notes = tempNotes.toList()



                if (Robot.isAutonomous) {
                    for (n in notes) {
                        for (s in noteList) {
                            if ((n.fieldCords.x - s.value.position.x).absoluteValue > 1.5 && (n.fieldCords.y - s.value.position.y).absoluteValue > 1.5) {
                                s.value.isPresent = true
                            } else {
                                s.value.isPresent = false
                            }
                        }
                    }
                }
                noteList[0]?.let { noteOnePresentEntry.setBoolean(it.isPresent) }
                noteList[1]?.let { noteOnePresentEntry.setBoolean(it.isPresent) }
                noteList[2]?.let { noteTwoPresentEntry.setBoolean(it.isPresent) }
                noteList[3]?.let { noteThreePresentEntry.setBoolean(it.isPresent) }
                noteList[4]?.let { noteFourPresentEntry.setBoolean(it.isPresent) }
            }
        }
    }

    fun getTargetRobotCords(target : PhotonTrackedTarget): Vector2 {
        val distance = distanceCurve.getValue(target.pitch).inches
        val xOffset = (distance.asInches * -tan(target.yaw.degrees)).inches
        return Vector2(distance.asFeet, xOffset.asFeet) + camRobotCoords
    }

    fun robotCordsToFieldCords(robotCords : Vector2): Vector2 {
        return robotCords.rotateDegrees(Drive.heading.asDegrees) + Drive.combinedPosition
    }
}

data class Note(
    val robotCords: Vector2,
    val fieldCords: Vector2,
    val timestampSeconds: Double
)
data class SchrodingerNote (
    val position: Vector2,
    var isPresent: Boolean = false
)