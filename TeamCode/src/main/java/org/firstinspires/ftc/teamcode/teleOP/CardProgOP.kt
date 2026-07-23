package org.firstinspires.ftc.teamcode.teleOP

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.customOpMode.OutReachOpMode
import org.firstinspires.ftc.teamcode.subSystems.TagSS

@TeleOp(name = "Card Programming TeleOP", group = "TeleOP")
class CardProgOP: OutReachOpMode() {
    override fun onInit() {
        tagSS = TagSS(hardwareMap)
    }

    override fun onLoop() {
        getDebugUtil().showTempDebug(
           tagSS
        )
        getDebugUtil().update(telemetry)
    }
}