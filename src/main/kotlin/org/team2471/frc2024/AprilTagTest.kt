package org.team2471.frc2024

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.photonvision.PhotonCamera
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.motion.following.SwerveDrive
import org.team2471.frc.lib.units.degrees

object AprilTagTest : Subsystem("AprilTag") {


    private val pvTable = NetworkTableInstance.getDefault().getTable("photonvision")

    private val outputTable = NetworkTableInstance.getDefault().getTable("AprilTags")
    private val wpiCoordsPose = outputTable.getEntry("wpiCoords")

    private val aprilTagFieldLayout : AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)

    var backRightCamera: PhotonCamera = PhotonCamera("Arducam_OV9281_USB_Camera") // Change name

    var pose : SwerveDrive.Pose? = null

    val cameras : List<Camera> = listOf(
        Camera(
            name = "Back Right Camera",
            camToRobot = Transform3d(-9.4095, -6.624, 7.075, Rotation3d(0.0, 20.0, -170.0)),
            photonCamera = PhotonCamera("backrightcam")
        ),
        Camera(
            name = "Center Camera",
            camToRobot = Transform3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0)),
            photonCamera = PhotonCamera("centercam")
        )
    )

//    -9.4095 x
//    -6.624 y
//    -7.075 z
//    Change
//    10 degree yaw to right
//    20 degree pitch up
//    no roll
//    from back right swerve


//    var photonPoseEstimator : PhotonPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, Transform3d(0.0, 0.0, 0.0, Rotation3d(0.0, 5.0, 0.0)))



    init {

        GlobalScope.launch {
            periodic {

                var bestPose : SwerveDrive.Pose? = null
                var numTags : Int = 0
                var ambiguity : Double? = null

                for(cam : Camera in cameras) {
                    val res = cam.photonCamera.latestResult
                    val multiRes = res.multiTagResult.estimatedPose

                    if (res.targets.size > numTags) {
                        if (multiRes.isPresent) {
                            if (res.targets.size > numTags || ambiguity == null || ambiguity > multiRes.ambiguity) {
                                bestPose = SwerveDrive.Pose(
                                    Vector2(
                                        multiRes.best.x,
                                        multiRes.best.y
                                    ), multiRes.best.rotation.angle.degrees
                                )
                                numTags = res.targets.size
                                ambiguity = multiRes.ambiguity
                            }
                        } else if (res.hasTargets() ) {
                            val best = res.bestTarget
                            val pose = aprilTagFieldLayout.getTagPose(best.fiducialId).get().transformBy(best.bestCameraToTarget.inverse()).transformBy(cam.camToRobot)
                            bestPose = SwerveDrive.Pose(Vector2(
                                pose.x,
                                pose.y
                            ), pose.rotation.angle.degrees)
                            numTags = 1
                            ambiguity = best.poseAmbiguity

                        }
                    }

                }

                if (bestPose != null) {
                    wpiCoordsPose.setDoubleArray(doubleArrayOf(
                        bestPose.position.x,
                        bestPose.position.y,
                        bestPose.heading.asDegrees
                    ))
                }

                pose = bestPose

            }
        }
    }


}

data class Camera(
    val name : String,
    val photonCamera : PhotonCamera,
    val camToRobot : Transform3d,
)