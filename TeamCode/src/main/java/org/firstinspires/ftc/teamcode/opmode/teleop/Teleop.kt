package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

@Configurable
@TeleOp
class Teleop : TeleOpMode() {
    override val alliance = VariableStateUtil.alliance
    override val opmode = Robot.OpMode.TELEOP

    override fun onInit() {
        robot.initPedro(hardwareMap, opmode)
    }

    override fun onStart() {
        //robot.follower.startTeleopDrive()
    }

    override fun onLoop() {
        //robot.follower.update()
    }
}