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
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.Angle.Companion.sin
import org.team2471.frc.lib.units.Angle.Companion.tan
import org.team2471.frc.lib.units.degrees

object NoteDetector: Subsystem("NoteDetector") {

    val pvtable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val camera : PhotonCamera = PhotonCamera("notecam")

    private val noteHeight = 1.0
    private val camHeight = 4.0

    private val cameraAngle = 0.0.degrees

    var notes : List<Note> = listOf()

    val seesNotes
        get() = notes.size > 0

    init {

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {

                val tempNotes : ArrayList<Note> = arrayListOf()

                for (target : PhotonTrackedTarget in camera.latestResult.targets) {
                    tempNotes.add(Note(
                        getTargetRobotCoords(target),
                        target.yaw.degrees
                    ))
                }

                notes = tempNotes.toList()
            }
        }
    }

    fun getTargetRobotCoords(target : PhotonTrackedTarget): Vector2 {
        val distance = (camHeight - noteHeight) * tan(target.pitch.degrees + cameraAngle)
        val xOffset = distance * tan(target.yaw.degrees)
        return Vector2(distance, xOffset)
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