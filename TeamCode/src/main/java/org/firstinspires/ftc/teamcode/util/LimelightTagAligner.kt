package org.firstinspires.ftc.teamcode.util

import com.pedropathing.follower.Follower
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * Simple "line up" helper for Limelight AprilTags.
 * - Finds a specific tag ID in fiducial results
 * - Rotates robot until tx ~= 0 (centered)
 */
class LimelightTagAligner(
    private val limelight: Limelight3A,
    private val tagId: Int,
) {
    // Tunables
    var maxFollowerPower: Double = 0.10
    var kP: Double = 0.03                 // rotate power per degree of tx
    var deadbandDeg: Double = 1.0         // degrees
    var maxRotate: Double = 0.30
    var maxStaleMs: Long = 150            // ignore old data

    /**
     * Call every loop. Returns true when aligned (inside deadband).
     * If tag not seen, it stops the drivetrain command (0,0,0).
     */
    fun update(follower: Follower): Boolean {
        follower.setMaxPower(maxFollowerPower)

        val fid = getTagFiducial(tagId)
        if (fid == null) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return false
        }

        val tx = fid.targetXDegrees   // left/right error in degrees

        if (abs(tx) <= deadbandDeg) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return true
        }

        val rotate = clip(tx * kP, -maxRotate, maxRotate)

        // Your old code used -rotationPower; keep same “feel” here:
        follower.setTeleOpDrive(0.0, 0.0, -rotate, false)
        return false
    }

    /** Optional: read current tx for telemetry/debug */
    fun getTxDegOrNull(): Double? = getTagFiducial(tagId)?.targetXDegrees

    private fun getTagFiducial(id: Int): LLResultTypes.FiducialResult? {
        val result: LLResult = limelight.latestResult ?: return null
        if (!result.isValid) return null
        if (result.staleness > maxStaleMs) return null

        // Find the desired tag
        val fiducials = result.fiducialResults ?: return null
        return fiducials.firstOrNull { it.fiducialId == id }
    }

    private fun clip(v: Double, lo: Double, hi: Double): Double {
        return max(lo, min(hi, v))
    }
}
