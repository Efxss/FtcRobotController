package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

class TeleOP : OpMode() {
    var panels: TelemetryManager? = null
    lateinit var lDrive: DcMotorEx
    lateinit var rDrive:  DcMotorEx
    override fun init() {
        lDrive = hardwareMap.get(DcMotorEx::class.java, "lDrive")
        rDrive = hardwareMap.get(DcMotorEx::class.java, "rDrive")
        rDrive.direction = DcMotorSimple.Direction.REVERSE
        panels?.debug("Init Started")
    }

    override fun loop() {
        var lD = gamepad1.left_stick_y
        var rD = gamepad1.right_stick_y
        if (lD >= 0.1) lDrive.power = 0.2; if (lD <= 0.1) lDrive.power = -0.2
        if (rD >= 0.1) rDrive.power = 0.2; if (rD <= 0.1) rDrive.power = -0.2
        panels?.apply {
            debug("Left Power", lDrive.power)
            debug("Right Power", rDrive.power)
            update(telemetry)
        }
    }
}