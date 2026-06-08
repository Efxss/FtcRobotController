package org.firstinspires.ftc.teamcode.config.subSystem

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.config.util.Alliance
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

class WebCamSS(
    hardwareMap : HardwareMap
) {
    private var aprilTag : AprilTagProcessor? = null
    private var visionPortal : VisionPortal? = null
    private val cameraPosition : Position = Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0)
    private val cameraOrientation = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0)
    private var cameraRes = Size(1280, 800)
    init { initAprilTag(hardwareMap) }
    fun getAprilTagCenterX(alliance : Alliance, deadzone : Double) : Double {
        var currentDetections : MutableList<AprilTagDetection?>? = aprilTag!!.detections
        if (currentDetections != null) {
            for (detection in currentDetections) {
                if (detection?.metadata != null) {
                    val targetId = when (alliance) {
                        Alliance.BLUE -> 20
                        Alliance.RED -> 24
                    }
                    if (detection.id != targetId) return 0.0
                    val tx = detection.center.x
                    return if (tx in 0.0 .. abs(deadzone)) 0.0 else -tx
                }
            }
        }
        return 0.0
    }
    private fun initAprilTag(hardwareMap : HardwareMap) {
        aprilTag = AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(908.758, 908.758, 696.345, 376.979)
                .build()
        val builder = VisionPortal.Builder()
        builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        builder.setCameraResolution(cameraRes)
        builder.addProcessor(aprilTag)
        visionPortal = builder.build()
    }
}