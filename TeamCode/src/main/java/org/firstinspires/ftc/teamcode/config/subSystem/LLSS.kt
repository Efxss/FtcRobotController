package org.firstinspires.ftc.teamcode.config.subSystem

import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.teamcode.config.Robot
import kotlin.math.abs
import kotlin.math.sign

class LLSS(
    robot: Robot
) {
    init {robot.ll.start()}

    fun getRotationPowerFromTag(alliance: Robot.Alliance, robot: Robot): Double {
        var kP = 0.025
        var minPower = 0.05
        var maxPower = 1.0
        var deadzone = 1.0
        val targetId = when (alliance) {
            Robot.Alliance.BLUE -> 20
            Robot.Alliance.RED -> 24
        }
        val result: LLResult? = robot.ll.latestResult
        if (result == null || !result.isValid) return 0.0
        val targetTag = result.fiducialResults.firstOrNull { it.fiducialId == targetId }
        if (targetTag == null) return 0.0
        val tx = targetTag.targetXDegrees
        if (abs(tx) < deadzone) return 0.0
        var power = kP * tx
        power = power.coerceIn(-maxPower, maxPower)
        if (abs(power) < minPower) power = minPower * sign(power)
        return -power
    }

    fun currentTagXDeg(alliance: Robot.Alliance, deadzone: Double, robot: Robot): Double {
        val targetId = when (alliance) {
            Robot.Alliance.BLUE -> 20
            Robot.Alliance.RED -> 24
        }
        val result: LLResult? = robot.ll.latestResult
        if (result == null || !result.isValid) return 0.0
        val targetTag = result.fiducialResults.firstOrNull { it.fiducialId == targetId }
        if (targetTag == null) return 0.0
        val td = targetTag.targetXDegrees
        return if (td in 0.0 .. abs(deadzone)) 0.0 else -td
    }

    fun currentTagXRad(alliance: Robot.Alliance, deadzone: Double, robot: Robot): Double {
        val targetId = when (alliance) {
            Robot.Alliance.BLUE -> 20
            Robot.Alliance.RED -> 24
        }
        val result: LLResult? = robot.ll.latestResult
        if (result == null || !result.isValid) return 0.0
        val targetTag = result.fiducialResults.firstOrNull { it.fiducialId == targetId }
        if (targetTag == null) return 0.0
        val tr = targetTag.targetXDegrees
        return if (tr in 0.0 .. abs(deadzone)) 0.0 else Math.toRadians(-tr)
    }

    fun isTagSeen(alliance: Robot.Alliance, robot: Robot): Boolean {
        val targetId = when (alliance) {
            Robot.Alliance.BLUE -> 20
            Robot.Alliance.RED -> 24
        }
        val result: LLResult? = robot.ll.latestResult
        if (result == null || !result.isValid) return false
        val targetTag = result.fiducialResults.firstOrNull { it.fiducialId == targetId }
        return targetTag != null
    }

    fun stop(robot: Robot) {
        robot.ll.stop()
    }
}