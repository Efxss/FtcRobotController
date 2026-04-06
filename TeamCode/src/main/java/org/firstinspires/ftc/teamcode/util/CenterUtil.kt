package org.firstinspires.ftc.teamcode.util

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

/** A utility script made for entering the centering sequence */
class CenterUtil (
    hardwareMap : HardwareMap,
    private val rotatePower : Double = 0.2,
    private val deadzone : Int = 15,
    private val id : Int = 20
) {
    private var rotationPower = 0.0f
    private val camWidthPx = 1280
    private val camHeightPx = 720
    private val kpRotate = 0.003
    private var isCentering = false


    private val driveUtil = DriveUtil(
        hardwareMap,
        drivePower = rotatePower
    )

    private val tagProcessor: AprilTagProcessor = AprilTagProcessor.Builder()
        .build()
    private val visionPortal: VisionPortal = VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        .setCameraResolution(Size(camWidthPx, camHeightPx))
        .addProcessor(tagProcessor)
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .build()


    /** Call this function to enter the centering sequence */
    fun centering(button: Boolean) {
        if (button) centerDepo() else if (isCentering) stopDrive()
    }

    fun centerDepo() {
        isCentering = true
        val detections = tagProcessor.detections
        val target = detections.firstOrNull { it.id == id }

        if (target == null) {
            stopDrive()
            return
        }

        val xErrPx = target.center.x - (camWidthPx / 2.0)

        if (abs(xErrPx) <= deadzone) {
            stopDrive()
            return
        }

        rotationPower = MathUtil.clip(xErrPx * kpRotate, -rotatePower, rotatePower).toFloat()
        driveUtil.setDrivePowers(-rotationPower.toDouble(), rotationPower.toDouble())
    }

    private fun stopDrive() {
        driveUtil.setDrivePowers(0.0, 0.0)
        isCentering = false
    }

    /** This function will return true if the robot is centering else it will return false */
    fun isCentering() : Boolean = isCentering

    /** This function will return the power of the robot during centering */
    fun getRotationPower() : Float  = rotationPower

    /** Calling this function will close everything safely */
    fun close() = visionPortal.close()
}