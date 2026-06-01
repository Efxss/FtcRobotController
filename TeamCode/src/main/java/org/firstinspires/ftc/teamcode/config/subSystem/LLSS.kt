package org.firstinspires.ftc.teamcode.config.subSystem

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.util.Alliance
import kotlin.math.abs
import kotlin.math.sign

class LLSS(
    hardwareMap: HardwareMap
) {
    private val ll : Limelight3A = hardwareMap.get(Limelight3A::class.java, "LL")
    init {ll.start()}

    fun getRotationPowerFromTag(alliance : Alliance) : Double {
        var kP = 0.025
        var minPower = 0.05
        var maxPower = 1.0
        var deadzone = 1.0
        val targetId = when (alliance) {
            Alliance.BLUE -> 20
            Alliance.RED -> 24
        }
        val result: LLResult? = ll.latestResult
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

    fun currentTagXDeg(alliance : Alliance, deadzone : Double) : Double {
        val targetId = when (alliance) {
            Alliance.BLUE -> 20
            Alliance.RED -> 24
        }
        val result: LLResult? = ll.latestResult
        if (result == null || !result.isValid) return 0.0
        val targetTag = result.fiducialResults.firstOrNull { it.fiducialId == targetId }
        if (targetTag == null) return 0.0
        val tx = targetTag.targetXDegrees
        return if (tx in 0.0 .. deadzone) 0.0 else -tx
    }

    fun currentTagXRad(alliance : Alliance, deadzone : Double) : Double {
        val targetId = when (alliance) {
            Alliance.BLUE -> 20
            Alliance.RED -> 24
        }
        val result: LLResult? = ll.latestResult
        if (result == null || !result.isValid) return 0.0
        val targetTag = result.fiducialResults.firstOrNull { it.fiducialId == targetId }
        if (targetTag == null) return 0.0
        val tx = targetTag.targetXDegrees
        return if (tx in 0.0 .. deadzone) 0.0 else Math.toRadians(-tx)
    }

    fun isTagSeen(alliance : Alliance) : Boolean {
        val targetId = when (alliance) {
            Alliance.BLUE -> 20
            Alliance.RED -> 24
        }
        val result: LLResult? = ll.latestResult
        if (result == null || !result.isValid) return false
        val targetTag = result.fiducialResults.firstOrNull { it.fiducialId == targetId }
        return targetTag != null
    }

    fun stop() {
        ll.stop()
    }
}