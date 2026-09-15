package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

@Configurable
@TeleOp
class Teleop : TeleOpMode() {
    override val alliance = VariableStateUtil.alliance

    override fun onInit() {
        robot.initializePedroPathing(hardwareMap)
    }

    override fun onStart() {
        robot.follower.startTeleopDrive()
    }

    override fun onLoop() {
        robot.follower.update()
    }
}