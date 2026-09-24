package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose

object PoseUtil {
    val p: PoseFactory = PoseFactory.degrees()
    val startPose: Pose = p.of(31.7, 8.0, Math.toRadians(180.0))
    val bottomLeftCorner: Pose = p.of(10.0, 8.0, Math.toRadians(180.0))
    val topLeftCorner: Pose = p.of(10.0, 132.0, Math.toRadians(180.0))
    val leftSpike: Pose = p.of(23.7, 85.0, Math.toRadians(90.0))
    val hiveFourLeftSide: Pose = p.of(56.0, 132.0, Math.toRadians(0.0))
}