package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx

class OuttakeController(
    private val out1: DcMotorEx,
    private val out2: DcMotorEx,
    private val tagId: Int
) {
    var lastPower: Double = 0.0
        private set

    fun update() {
        val p = LimelightUtil.calculateOuttakePower(tagId) ?: return
        lastPower = p
        out1.power = p
        out2.power = p
    }

    fun stop() {
        lastPower = 0.0
        out1.power = 0.0
        out2.power = 0.0
    }
}
