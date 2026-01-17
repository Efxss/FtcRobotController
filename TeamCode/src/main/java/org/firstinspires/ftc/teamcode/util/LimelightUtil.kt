package org.firstinspires.ftc.teamcode.util

import com.pedropathing.follower.Follower
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.delay
import kotlin.math.abs

/**
 * Limelight utility object for AprilTag tracking, centering, and ball dispensing
 * Usage: Call LimelightUtil.init(hardwareMap) in your OpMode's init()
 */
object LimelightUtil {

    // Hardware references
    private lateinit var limelight: Limelight3A
    private lateinit var bowlServo: Servo
    private lateinit var camServo: Servo

    // Configuration
    object Config {
        // Camera settings
        const val CAM_WIDTH_PX = 1280
        const val CAM_HEIGHT_PX = 960
        const val H_FOV_DEG = 70.0
        val H_FOV_RAD = Math.toRadians(H_FOV_DEG)
        val PIXELS_TO_RAD = H_FOV_RAD / CAM_WIDTH_PX

        // Centering thresholds
        const val CENTER_DEADZONE_PX = 15
        const val CENTER_TIMEOUT_MS = 500L

        // Servo positions
        const val CAM_OPEN = 0.44
        const val CAM_CLOSED = 0.255

        // Loading positions
        const val LOAD_P1 = 0.021
        const val LOAD_P2 = 0.087
        const val LOAD_P3 = 0.158

        // Firing/dispensing positions
        const val FIRE_P1 = 0.128
        const val FIRE_P2 = 0.195
        const val FIRE_P3 = 0.058

        // Timing
        const val DISPENSE_INITIAL_DELAY_MS = 100L
        const val BOWL_MOVE_DELAY_MS = 250L
        const val CAM_OPEN_DELAY_MS = 140L
        const val CAM_CLOSE_DELAY_MS = 170L

        // Poll rate
        const val POLL_RATE_HZ = 100
    }

    // Known AprilTag IDs
    object Tags {
        const val RED_DEPO = 24
        const val BLUE_DEPO = 21  // Add your blue depot tag if needed
        // Add more tags as needed
    }

    // Centering result
    data class CenterResult(
        val centered: Boolean,
        val errorPx: Double,
        val tagFound: Boolean
    )

    /**
     * Initialize the Limelight utility with hardware references
     * Call this in your OpMode's init() method
     */
    fun init(hardwareMap: HardwareMap,
             limelightName: String = "limelight",
             bowlServoName: String = "bowlServo",
             camServoName: String = "camServo",
             pipeline: Int = 0) {

        limelight = hardwareMap.get(Limelight3A::class.java, limelightName)
        bowlServo = hardwareMap.get(Servo::class.java, bowlServoName)
        camServo = hardwareMap.get(Servo::class.java, camServoName)

        // Initialize servo positions
        camServo.position = Config.CAM_CLOSED
        bowlServo.position = Config.LOAD_P1

        // Setup Limelight
        limelight.setPollRateHz(Config.POLL_RATE_HZ)
        limelight.pipelineSwitch(pipeline)
        limelight.start()
    }

    /**
     * Initialize with existing hardware references (if you already have them)
     */
    fun init(ll: Limelight3A, bowl: Servo, cam: Servo) {
        limelight = ll
        bowlServo = bowl
        camServo = cam
    }

    /**
     * Stop the Limelight - call in OpMode's stop()
     */
    fun stop() {
        if (::limelight.isInitialized) {
            limelight.stop()
        }
    }

    /**
     * Switch Limelight pipeline
     */
    fun setPipeline(pipeline: Int) {
        limelight.pipelineSwitch(pipeline)
    }

    // ==================== TAG DETECTION ====================

    /**
     * Check if a specific AprilTag is visible
     */
    fun isTagVisible(tagId: Int): Boolean {
        val result = limelight.latestResult ?: return false
        if (!result.isValid) return false
        return result.fiducialResults.any { it.fiducialId == tagId }
    }

    /**
     * Get the pixel error from center for a specific tag
     * Returns null if tag not found
     */
    fun getTagXError(tagId: Int): Double? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null

        val target = result.fiducialResults.firstOrNull { it.fiducialId == tagId } ?: return null
        return target.targetXPixels - (Config.CAM_WIDTH_PX / 2.0)
    }

    /**
     * Get the Y pixel position for a specific tag (useful for distance estimation)
     */
    fun getTagYPixels(tagId: Int): Double? {
        val result = limelight.latestResult ?: return null
        if (!result.isValid) return null

        val target = result.fiducialResults.firstOrNull { it.fiducialId == tagId } ?: return null
        return target.targetYPixels
    }

    // ==================== CENTERING ====================

    /**
     * Attempt to center on a specific AprilTag
     * Returns CenterResult with centering status
     *
     * @param tagId The AprilTag ID to center on
     * @param follower The Pedro Pathing follower
     * @param deadzone Custom deadzone in pixels (optional)
     * @return CenterResult containing centering status
     */
    fun centerOnTag(
        tagId: Int,
        follower: Follower,
        deadzone: Int = Config.CENTER_DEADZONE_PX
    ): CenterResult {
        val xError = getTagXError(tagId) ?: return CenterResult(
            centered = false,
            errorPx = Double.MAX_VALUE,
            tagFound = false
        )

        if (abs(xError) <= deadzone) {
            return CenterResult(centered = true, errorPx = xError, tagFound = true)
        }

        // Calculate angle correction
        val angleError = xError * Config.PIXELS_TO_RAD
        val currentHeading = follower.pose.heading
        val targetHeading = currentHeading - angleError

        follower.turnTo(targetHeading)

        return CenterResult(centered = false, errorPx = xError, tagFound = true)
    }


    /**
     * Attempt to always center on a specific AprilTag
     * Returns CenterResult with centering status
     *
     * @param tagId The AprilTag ID to center on
     * @param follower The Pedro Pathing follower
     * @param deadzone Custom deadzone in pixels (optional)
     * @return CenterResult containing centering status
     */

    fun constantCenterOnTag(
        tagId: Int,
        follower: Follower,
        deadzone: Int = Config.CENTER_DEADZONE_PX
    ): CenterResult {
        val xError = getTagXError(tagId) ?: return CenterResult(
            centered = false,
            errorPx = Double.MAX_VALUE,
            tagFound = false
        )

        // Calculate angle correction
        val angleError = xError * Config.PIXELS_TO_RAD
        val currentHeading = follower.pose.heading
        val targetHeading = currentHeading - angleError

        follower.turnTo(targetHeading)

        return CenterResult(centered = false, errorPx = xError, tagFound = true)
    }

    /**
     * Blocking center on tag - keeps trying until centered or timeout
     * Use this in a coroutine context
     *
     * @param tagId The AprilTag ID to center on
     * @param follower The Pedro Pathing follower
     * @param timeoutMs Maximum time to attempt centering
     * @param checkIntervalMs How often to check centering status
     * @return true if successfully centered, false if timed out
     */
    suspend fun centerOnTagBlocking(
        tagId: Int,
        follower: Follower,
        timeoutMs: Long = Config.CENTER_TIMEOUT_MS,
        checkIntervalMs: Long = 20L
    ): Boolean {
        val startTime = System.currentTimeMillis()

        while (System.currentTimeMillis() - startTime < timeoutMs) {
            val result = centerOnTag(tagId, follower)
            if (result.centered) {
                return true
            }
            delay(checkIntervalMs)
        }

        return false
    }

    // ==================== FIRING ====================

    /**
     * Fire a specific number of balls
     *
     * @param ballCount Number of balls to fire (1-3)
     */
    suspend fun fire(ballCount: Int) {
        val positions = when (ballCount) {
            1 -> listOf(Config.FIRE_P2)  // Fire from slot 1 (middle)
            2 -> listOf(Config.FIRE_P2, Config.FIRE_P1)  // Fire slots 1, 0
            3 -> listOf(Config.FIRE_P2, Config.FIRE_P1, Config.FIRE_P3)  // Fire all
            else -> return
        }

        executeDispenseSequence(positions)
        bowlServo.position = Config.LOAD_P1
    }

    /**
     * Fire balls from specific slots based on what's loaded
     *
     * @param slots Array of slot states ("N" = empty, anything else = loaded)
     * @param dispenseOrder Order to dispense slots (default: 1, 0, 2)
     */
    suspend fun fireLoaded(
        slots: Array<String>,
        dispenseOrder: List<Int> = listOf(1, 0, 2)
    ) {
        val slotToFirePosition = mapOf(
            0 to Config.FIRE_P1,
            1 to Config.FIRE_P2,
            2 to Config.FIRE_P3
        )

        val dispenseSequence = mutableListOf<Double>()

        for (slotIndex in dispenseOrder) {
            if (slots.getOrNull(slotIndex) != "N") {
                slotToFirePosition[slotIndex]?.let { dispenseSequence.add(it) }
            }
        }

        if (dispenseSequence.isNotEmpty()) {
            executeDispenseSequence(dispenseSequence)
        }

        bowlServo.position = Config.LOAD_P1
    }

    /**
     * Execute the dispense sequence for given bowl positions
     */
    private suspend fun executeDispenseSequence(positions: List<Double>) {
        delay(Config.DISPENSE_INITIAL_DELAY_MS)

        positions.forEach { position ->
            bowlServo.position = position
            delay(Config.BOWL_MOVE_DELAY_MS)

            camServo.position = Config.CAM_OPEN
            delay(Config.CAM_OPEN_DELAY_MS)

            camServo.position = Config.CAM_CLOSED
            delay(Config.CAM_CLOSE_DELAY_MS)
        }
    }

    // ==================== COMBINED OPERATIONS ====================

    /**
     * Center on a tag and then fire all loaded balls
     *
     * @param tagId The AprilTag ID to center on
     * @param follower The Pedro Pathing follower
     * @param slots Array of slot states
     * @param centerTimeoutMs Timeout for centering
     * @return true if successfully centered and fired
     */
    suspend fun centerAndFire(
        tagId: Int,
        follower: Follower,
        slots: Array<String>,
        centerTimeoutMs: Long = Config.CENTER_TIMEOUT_MS
    ): Boolean {
        val centered = centerOnTagBlocking(tagId, follower, centerTimeoutMs)

        // Fire regardless of centering success (you may want to change this)
        fireLoaded(slots)

        return centered
    }

    /**
     * Center on a tag and fire a specific number of balls
     *
     * @param tagId The AprilTag ID to center on
     * @param follower The Pedro Pathing follower
     * @param ballCount Number of balls to fire
     * @param centerTimeoutMs Timeout for centering
     * @return true if successfully centered
     */
    suspend fun centerAndFire(
        tagId: Int,
        follower: Follower,
        ballCount: Int,
        centerTimeoutMs: Long = Config.CENTER_TIMEOUT_MS
    ): Boolean {
        val centered = centerOnTagBlocking(tagId, follower, centerTimeoutMs)

        fire(ballCount)

        return centered
    }

    // ==================== OUTTAKE POWER CALCULATION ====================

    /**
     * Calculate outtake power based on tag Y position
     * Returns null if tag not visible
     *
     * @param tagId The AprilTag ID to use for distance
     * @return Calculated power value or null
     */
    fun calculateOuttakePower(tagId: Int): Double? {
        val targetY = getTagYPixels(tagId) ?: return null
        return 0.0002416 * targetY + 0.105
    }

    // ==================== BOWL MANAGEMENT ====================

    /**
     * Advance bowl to the next loading position
     *
     * @param currentSlot The slot that was just filled (0, 1, or 2)
     */
    fun advanceBowl(currentSlot: Int) {
        bowlServo.position = when (currentSlot) {
            0 -> Config.LOAD_P2
            1 -> Config.LOAD_P3
            2 -> Config.FIRE_P2
            else -> bowlServo.position
        }
    }

    /**
     * Reset bowl to initial loading position
     */
    fun resetBowl() {
        bowlServo.position = Config.LOAD_P1
    }

    /**
     * Open the cam servo
     */
    fun openCam() {
        camServo.position = Config.CAM_OPEN
    }

    /**
     * Close the cam servo
     */
    fun closeCam() {
        camServo.position = Config.CAM_CLOSED
    }
}