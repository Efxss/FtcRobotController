package org.firstinspires.ftc.teamcode.teleOP

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.util.CenterUtil
import org.firstinspires.ftc.teamcode.util.DriveUtil

@TeleOp(name = "TeleOP", group = "Main TeleOP")
class TeleOP : OpMode() {
    var panels: TelemetryManager? = null
    private lateinit var driveUtil: DriveUtil
    private lateinit var centerUtil: CenterUtil
    override fun init() {
        panels = PanelsTelemetry.telemetry
        driveUtil = DriveUtil(hardwareMap, drivePower = 0.6, deadzone = 0.2f)
        centerUtil = CenterUtil(hardwareMap, 0.2, 15, 20)
        panels?.debug("Init Started")
        panels?.update(telemetry)
    }

    override fun loop() {
        driveUtil.tankDrive(
            gamepad1.left_stick_y,
            gamepad1.right_stick_y
        )
        centerUtil.centering(gamepad1.cross)
        panels?.update(telemetry)
    }
}