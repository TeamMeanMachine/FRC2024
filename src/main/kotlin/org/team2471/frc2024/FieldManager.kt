package org.team2471.frc2024

import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.math.*


val reefTagIds = intArrayOf(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)

val reefPoints = AprilTag.aprilTagFieldLayout.tags.filter{it.ID in reefTagIds}.map {Vector2L(it.pose.x.meters, it.pose.y.meters) + Vector2L(1.5.feet, 0.0.feet).rotateRadians(it.pose.rotation.z)}