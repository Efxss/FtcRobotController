package org.firstinspires.ftc.teamcode.config.subSystem

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.config.util.Alliance
import org.firstinspires.ftc.teamcode.config.util.MathUtil

class LLSS(
    hardwareMap: HardwareMap
) {
    private val ll : Limelight3A = hardwareMap.get(Limelight3A::class.java, "LL")
    init {ll.start()}

    fun getRotationPowerFromTag(alliance: Alliance) : Double {
        val xErrPx = ll.latestResult
            ?.fiducialResults
            ?.firstOrNull { it.fiducialId == when (alliance) {
                Alliance.BLUE -> 20
                Alliance.RED -> 24
            } }
            ?.targetXPixels
            ?: return 0.0

        return MathUtil.clip(xErrPx, -1.0, 1.0)
    }

    fun stop() {
        ll.stop()
    }
}