package org.firstinspires.ftc.teamcode.subSystems

import android.util.Size
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.util.VariableStateUtil
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class TagSS(
    hardwareMap : HardwareMap,
    webcamName: String = "Webcam 1"
) {
    companion object { const val MAX_TAGS = 6 }
    private var aprilTag: AprilTagProcessor? = null
    private var visionPortal: VisionPortal? = null
    private val cameraPosition: Position = Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0)
    private val cameraOrientation = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0)
    private val cameraRes = Size(640, 480)
    init {initAprilTag(hardwareMap, webcamName)}
    private var lastSeenId = 0
    private var wasSeen = false
    private var firstTime = true
    var lastRuntime: Double = 0.0
    var resetRuntime = false
    fun update(runtime: Double) {
        val seen = aprilTag?.detections?.firstOrNull { it?.metadata != null }
        if (seen != null && !wasSeen && VariableStateUtil.tagList.size < MAX_TAGS) {
            if (lastRuntime != runtime && !resetRuntime) {
                lastRuntime = runtime
                resetRuntime = true
            }
            if (runtime - lastRuntime > 2.0 || firstTime) {
                VariableStateUtil.tagList.add(seen.id)
                resetRuntime = false
                firstTime = false
            }
        }
        wasSeen = seen != null
        lastSeenId = seen?.id ?: 0
    }
    fun clearList(): Command = Commands.instant{ VariableStateUtil.tagList.clear()
        wasSeen = false
        resetRuntime = false
        firstTime = true }
    fun tagListDat(): MutableList<Int> = VariableStateUtil.tagList
    fun tagListSize(): Int = VariableStateUtil.tagList.size
    fun lastRuntimeDat(): Double = lastRuntime
    fun resetRuntimeDat(): Boolean = resetRuntime
    fun firstTimeDat(): Boolean = firstTime
    private fun initAprilTag(hardwareMap: HardwareMap, webcamName: String) {
        aprilTag = AprilTagProcessor.Builder()
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .setCameraPose(cameraPosition, cameraOrientation)
            .build()
        val builder = VisionPortal.Builder()
        builder.setCamera(hardwareMap.get(WebcamName::class.java, webcamName))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        builder.setCameraResolution(cameraRes)
        builder.addProcessor(aprilTag)
        visionPortal = builder.build()
    }
    fun stop() { visionPortal?.close() }
}