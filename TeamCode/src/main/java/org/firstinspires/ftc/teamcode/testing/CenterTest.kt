package org.firstinspires.ftc.teamcode.testing

import android.util.Size
import com.bylazar.configurables.annotations.IgnoreConfigurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

@Disabled
class CenterTest : OpMode() {
    @IgnoreConfigurable
    var panels: TelemetryManager? = null
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer
    private var visionPortal: VisionPortal? = null
    private var tagProcessor: AprilTagProcessor? = null
    private val startPose    = Pose(72.0, 0.0, Math.toRadians(90.0))
    object AprilTagIds {
        const val GPP_ORDER = 21
        const val PGP_ORDER = 22
        const val PPG_ORDER = 23
        const val RED_DEPO =  24
    }
    object DepoCenter {
        const val DESIRED_TAG_WIDTH_PX = 110
        const val ROTATE_POWER = 0.2
        const val CAM_WIDTH_PX = 1280
        const val CAM_HEIGHT_PX = 720
        const val CENTER_DEADZONE = 15
        const val KP_ROTATE = 0.003
    }
    private enum class PieceColor(val symbol: String) {
        NONE("N"),
        GREEN("G"),
        PURPLE("P");

        companion object {
            fun fromString(s: String): PieceColor =
                entries.find { it.symbol == s } ?: NONE  // Changed from values() to entries
        }
    }
    private data class GamePieceOrder(
        var slot1: PieceColor = PieceColor.NONE,
        var slot2: PieceColor = PieceColor.NONE,
        var slot3: PieceColor = PieceColor.NONE
    ) {
        fun isFull() = slot1 != PieceColor.NONE &&
                slot2 != PieceColor.NONE &&
                slot3 != PieceColor.NONE

        fun isEmpty() = slot1 == PieceColor.NONE &&
                slot2 == PieceColor.NONE &&
                slot3 == PieceColor.NONE

        fun contains(color: PieceColor) = slot1 == color || slot2 == color || slot3 == color

        fun addPiece(color: PieceColor): Boolean {
            //if (contains(color)) return false

            when {
                slot1 == PieceColor.NONE -> { slot1 = color; return true }
                slot2 == PieceColor.NONE -> { slot2 = color; return true }
                slot3 == PieceColor.NONE -> { slot3 = color; return true }
            }
            return false
        }

        fun reset() {
            slot1 = PieceColor.NONE
            slot2 = PieceColor.NONE
            slot3 = PieceColor.NONE
        }

        fun matches(expected: GamePieceOrder) =
            slot1 == expected.slot1 && slot2 == expected.slot2 && slot3 == expected.slot3

        override fun toString() = "${slot1.symbol}${slot2.symbol}${slot3.symbol}"
    }
    private val currentOrder = GamePieceOrder()
    private val expectedOrder = GamePieceOrder()
    private var currentLoadPosition = 1
    override fun init() {
        setupVision()
        initPedroPathing()
    }
    override fun start() {
        opmodeTimer.resetTimer()
    }
    override fun loop() {
        follower.update()
        centerDepo()
        processAprilTags()
        panels?.debug("EORD", expectedOrder)
        panels?.update(telemetry)
    }
    private fun setupVision() {
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults()
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(tagProcessor)
            .setCameraResolution(Size(1280, 720))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
    }
    private fun initPedroPathing() {
        panels = PanelsTelemetry.telemetry
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
    }

    private fun centerDepo() {
        val detections = tagProcessor?.detections.orEmpty()

        // Find the RED_DEPO tag
        val target = detections.firstOrNull { it.id == AprilTagIds.RED_DEPO }

        if (target == null) {
            panels?.debug("RED_DEPO tag not in view")
            panels?.update(telemetry)
            return
        }

        println("Function is running")
        panels?.debug("function is running")

        // Horizontal pixel error from image center
        val xErrPx: Double = target.center.x - (DepoCenter.CAM_WIDTH_PX / 2.0)

        // Tag width in pixels (you can use this later for distance control)
        val tagWidthPx = hypot(
            target.corners[1].x - target.corners[0].x,
            target.corners[1].y - target.corners[0].y
        )
        val widthErrPx = DepoCenter.DESIRED_TAG_WIDTH_PX - tagWidthPx

        // If we're close enough to the center, don't command any more turns
        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            panels?.debug("Centered on tag")
            panels?.debug("xErrPx", xErrPx)
            panels?.debug("tagWidthPx", tagWidthPx)
            panels?.debug("widthErrPx", widthErrPx)
            panels?.update(telemetry)
            return
        }

        // --- Pixel error -> angle error ---

        // Approximate horizontal FOV of your webcam (tune this if needed)
        val hFovDeg = 70.0
        val hFovRad = Math.toRadians(hFovDeg)

        // How many radians of angle correspond to 1 pixel of x error
        val pixelsToRad = hFovRad / DepoCenter.CAM_WIDTH_PX

        // Angle error in radians (how far off-center the tag is)
        val angleError = xErrPx * pixelsToRad

        // Limit how aggressively we change the target heading each loop
        val maxTurnStepDeg = 10.0
        val maxTurnStepRad = Math.toRadians(maxTurnStepDeg)

        val turnStep = clip(
            angleError,
            -maxTurnStepRad,
            maxTurnStepRad
        )

        val currentHeading = follower.pose.heading

        // If the tag is to the right (xErrPx > 0), we want to turn right (negative angle),
        // so we SUBTRACT the step from the current heading.
        val targetHeading = currentHeading - turnStep

        follower.turnTo(targetHeading)

        panels?.debug("Tag in view")
        panels?.debug("xErrPx", xErrPx)
        panels?.debug("tagWidthPx", tagWidthPx)
        panels?.debug("widthErrPx", widthErrPx)
        panels?.debug("angleErrorRad", angleError)
        panels?.debug("turnStepRad", turnStep)
        panels?.debug("targetHeading", targetHeading)
        panels?.update(telemetry)
    }

    private fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }

    private fun processAprilTags() {
        val detections = tagProcessor?.detections.orEmpty()

        val orderTag = detections.firstOrNull { it.id in listOf(
            AprilTagIds.GPP_ORDER,
            AprilTagIds.PGP_ORDER,
            AprilTagIds.PPG_ORDER
        ) }

        orderTag?.let { tag ->
            when (tag.id) {
                AprilTagIds.GPP_ORDER -> {
                    expectedOrder.slot1 = PieceColor.GREEN
                    expectedOrder.slot2 = PieceColor.PURPLE
                    expectedOrder.slot3 = PieceColor.PURPLE
                    panels?.debug("Detected Tag 21: GPP")
                    panels?.update(telemetry)
                }
                AprilTagIds.PGP_ORDER -> {
                    expectedOrder.slot1 = PieceColor.PURPLE
                    expectedOrder.slot2 = PieceColor.GREEN
                    expectedOrder.slot3 = PieceColor.PURPLE
                    panels?.debug("Detected Tag 22: PGP")
                    panels?.update(telemetry)
                }
                AprilTagIds.PPG_ORDER -> {
                    expectedOrder.slot1 = PieceColor.PURPLE
                    expectedOrder.slot2 = PieceColor.PURPLE
                    expectedOrder.slot3 = PieceColor.GREEN
                    panels?.debug("Detected Tag 23: PPG")
                    panels?.update(telemetry)
                }
            }
        }
    }
}