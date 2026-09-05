package org.firstinspires.ftc.teamcode.config.subSystem

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.Robot
import kotlin.math.abs

class WebCamSS(
    hardwareMap: HardwareMap,
    robot: Robot,
    webcamName: String = "Webcam 1"
) {
    init { robot.initAprilTag(hardwareMap, webcamName) }
    fun currentTagXRad(alliance: Robot.Alliance, deadzone: Double, robot: Robot): Double {
        val targetId = when (alliance) {
            Robot.Alliance.BLUE -> 20
            Robot.Alliance.RED -> 24
        }
        val detection = robot.aprilTag?.detections?.firstOrNull { it.id == targetId && it.ftcPose != null } ?: return 0.0
        val tr = detection.ftcPose.bearing
        return if (tr in 0.0..abs(deadzone)) 0.0 else tr
    }
    fun isTagSeen(alliance: Robot.Alliance, robot: Robot): Boolean {
        val targetId = when (alliance) {
            Robot.Alliance.BLUE -> 20
            Robot.Alliance.RED -> 24
        }
        val detection = robot.aprilTag?.detections?.firstOrNull { it.id == targetId && it.ftcPose != null } ?: return false
        return detection.metadata != null
    }
    fun stop(robot: Robot) { robot.visionPortal?.close() }
}