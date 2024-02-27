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

object NoteDetector: Subsystem("NoteDetector") {

    private val pvtable = NetworkTableInstance.getDefault().getTable("photonvision")
    private val camera : PhotonCamera = PhotonCamera("notecam")
    private val noteAdvantagePosEntry = pvtable.getEntry("Advantage Note Pos")

    private val noteHalfHeight = 1.0.inches
    private val camHeight = 9.796.inches
    private val camRobotCoords = Vector2(0.0.inches.asFeet, 11.96.inches.asFeet)
    private val cameraAngle = 0.0.degrees

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