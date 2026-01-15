package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.delay

/**
 * Tracks ball detections, ord[] filling, cooldown, and which bowl servo position to advance to.
 * Keeps your same behavior:
 * - Clear condition: g<=80 && b<=110 => re-arm immediately
 * - Cooldown: 1500ms after a detection
 * - Classification: g>=110 => "G" else "P"
 * - After detection: delay(200), then advance bowl to slot position
 */
class LoaderTracker(
    initialOrd: Array<String> = arrayOf("N", "N", "N"),
    private val detectionCooldownMs: Long = 1500L,
    private val clearG: Int = 80,
    private val clearB: Int = 110,
    private val greenIsGThreshold: Int = 110
) {
    private val ord = initialOrd.copyOf()

    private var isSeen = false
    private var nextDetectAllowedMs = 0L

    data class DetectionResult(
        val slot: Int,
        val type: String,          // "G" or "P"
        val newBowlPosition: Double
    )

    fun snapshotOrd(): Array<String> = ord.copyOf()

    fun clearOrd(fill: String = "N") {
        for (i in ord.indices) ord[i] = fill
    }

    fun isFull(): Boolean = ord.none { it == "N" }

    /**
     * Suspend because we keep your original delay(200) after detection.
     * Returns a DetectionResult if something was detected and bowl should advance; otherwise null.
     */
    suspend fun updateFromSensor(g: Int, b: Int, nowMs: Long): DetectionResult? {
        // "Clear" => re-arm immediately (even during cooldown)
        if (g <= clearG && b <= clearB) {
            isSeen = false
            return null
        }

        // cooldown
        if (nowMs < nextDetectAllowedMs) return null

        if (!isSeen) {
            val slot = nextSlot()
            if (slot != -1) {
                val type = if (g >= greenIsGThreshold) "G" else "P"
                ord[slot] = type

                delay(200)

                val pos = bowlPositionForSlot(slot)
                nextDetectAllowedMs = nowMs + detectionCooldownMs
                isSeen = true

                return DetectionResult(slot, type, pos)
            }
            isSeen = true
        }

        return null
    }

    private fun nextSlot(): Int = when {
        ord[0] == "N" -> 0
        ord[1] == "N" -> 1
        ord[2] == "N" -> 2
        else -> -1
    }

    private fun bowlPositionForSlot(slot: Int): Double = when (slot) {
        0 -> TeleOpConfig.ServoPositions.LOAD_P2
        1 -> TeleOpConfig.ServoPositions.LOAD_P3
        2 -> TeleOpConfig.ServoPositions.FIRE_P2
        else -> TeleOpConfig.ServoPositions.FIRE_P2
    }
}
