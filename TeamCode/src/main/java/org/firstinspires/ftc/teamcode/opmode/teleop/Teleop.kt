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
        robot.initPedro()
        flowerSS.reset()
    }

    override fun onStart() {
        robot.follower.manual(dp)
        // ManualDrive.driveOrHold(robot.follower, dp) // Do this to make it so when you stop holding the stick it holds position
    }

    override fun onLoop() {
        robot.follower.update()
    }
}