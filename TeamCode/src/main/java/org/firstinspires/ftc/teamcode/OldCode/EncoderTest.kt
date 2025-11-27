package org.firstinspires.ftc.teamcode.OldCode

import com.bylazar.configurables.annotations.IgnoreConfigurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

@Disabled
class EncoderTest : OpMode() {
    @IgnoreConfigurable
    var panels: TelemetryManager? = null
    private lateinit var frontLeft: DcMotorEx
    private lateinit var frontRight: DcMotorEx
    private lateinit var backLeft: DcMotorEx
    private lateinit var backRight: DcMotorEx
    override fun init() {
        panels = PanelsTelemetry.telemetry
        frontLeft = hardwareMap.get(DcMotorEx::class.java, "frontLeft")
        frontRight = hardwareMap.get(DcMotorEx::class.java, "frontRight")
        backLeft = hardwareMap.get(DcMotorEx::class.java, "backLeft")
        backRight = hardwareMap.get(DcMotorEx::class.java, "backRight")
        resetEncoders()
    }

    override fun loop() {
        panels?.apply { addData("frontLeft Encoder", frontLeft.currentPosition); addData("frontRight Encoder", frontRight.currentPosition); addData("backLeft Encoder", backLeft.currentPosition); addData("backRight Encoder", backRight.currentPosition); update() }
    }

    private fun resetEncoders() {
        listOf(frontLeft, frontRight, backLeft, backRight).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
}