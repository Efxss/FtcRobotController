package org.firstinspires.ftc.teamcode.config.subSystem

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.util.MathUtil
import kotlin.math.abs

class LLSS(
    hardwareMap: HardwareMap
) {
    private val ll : Limelight3A = hardwareMap.get(Limelight3A::class.java, "LL")
    init {ll.start()}

    fun getTagXError() : Double? {
        val result: LLResult? = ll.latestResult
        val fiducialResults = result?.fiducialResults
        val target = fiducialResults?.firstOrNull { it.fiducialId == 20 }
        return target?.targetXPixels?.let { abs(it) }
    }

    fun getRotationPowerFromTag() : Double {
        val result : LLResult? = ll.latestResult
        val fiducialResults = result?.fiducialResults
        val target = fiducialResults?.firstOrNull { it.fiducialId == 20 }
        val xErrPx = target?.targetXPixels
        return MathUtil.clip(xErrPx!!, -1.0, 1.0)
    }

    fun stop() {
        ll.stop()
    }
}