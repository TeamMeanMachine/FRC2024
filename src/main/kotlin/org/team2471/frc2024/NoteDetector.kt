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
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.Angle.Companion.sin
import org.team2471.frc.lib.units.Angle.Companion.tan
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches

object NoteDetector: Subsystem("NoteDetector") {

    val pvtable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val camera : PhotonCamera = PhotonCamera("notecam")
    private val noteAdvantagePosEntry = pvtable.getEntry("Advantage Note Pos")

    private val noteHalfHeight = 1.0.inches
    private val camHeight = 9.796.inches
    private val camRobotCoords = Vector2(0.0.inches.asMeters, 11.96.inches.asMeters)

    val distanceCurve = MotionCurve()

    private val cameraAngle = 0.0.degrees

    var notes : List<Note> = listOf()

    var notePosAdv: MutableList<Array<Double>> = mutableListOf()

    val seesNotes
        get() = notes.size > 0

    init {
        distanceCurve.storeValue(-1.6, 109.0)
        distanceCurve.storeValue(-17.5, 12.0)
        distanceCurve.storeValue(-6.48, 46.0)
        distanceCurve.storeValue(-3.5, 73.0)



        GlobalScope.launch(MeanlibDispatcher) {
            periodic {

                val tempNotes : ArrayList<Note> = arrayListOf()

                notePosAdv = mutableListOf()

                if (camera.latestResult.targets.isNotEmpty()) {
                    for (target : PhotonTrackedTarget in camera.latestResult.targets) {
                        val notePos = getTargetRobotCoords(target)

                        tempNotes.add(Note(
                            notePos,
                            target.yaw.degrees
                        ))
//                    println("notePose: $notePos  combinedPose: ${Drive.combinedPosition}")
//                    println("distance curve: ${distanceCurve.getValue(target.pitch)}")

                        val noteRotPos = notePos.rotateDegrees(Drive.heading.asDegrees)

                        notePosAdv.add(arrayOf(Drive.combinedPosition.x + noteRotPos.x, Drive.combinedPosition.y + noteRotPos.y, 0.0))
                    }
                }


//                println("Hi: ${notePosAdv[0]}")
                if (notePosAdv.isNotEmpty()) {
                    noteAdvantagePosEntry.setDoubleArray(notePosAdv.first())
                } else {
                    noteAdvantagePosEntry.setDoubleArray(doubleArrayOf(0.0))
                }


                notes = tempNotes.toList()
            }
        }
    }

    fun getTargetRobotCoords(target : PhotonTrackedTarget): Vector2 {
//        println("pitch: ${ target.pitch.degrees }")
        val distance = distanceCurve.getValue(target.pitch).inches//((camHeight - noteHalfHeight).asInches / -tan(target.pitch.degrees + cameraAngle)).inches
//        println("distance: ${distance.asFeet.round(4)}   pitch: ${target.pitch.round(4)}")
        val xOffset = (distance.asInches * -tan(target.yaw.degrees)).inches
    return Vector2(distance.asMeters, xOffset.asMeters) + camRobotCoords
    }

//    fun getNoteCoords(): Vector2 {
//        val angleFromBottom = boundingBoxLowerY/ limelightScreenHeight * NoteLimelight.limelightVerticalFOV
//        val tangentAngle = (NoteLimelight.limelightVerticalFOV /2 - angleFromBottom) * Math.PI/180.0
//        val distance = NoteLimelight.noteDistFromCenter + 1/12 * (sin(tangentAngle) + ((NoteLimelight.limelightHeight - (1- cos(tangentAngle)))/ tan(tangentAngle)))
//        return NoteLimelight.limelightToRobotCoords(Vector2(distance / tan(xDegreeOffset), distance))
//    }

}

data class Note(
    val robotCoords : Vector2,
    val degOffset : Angle
)