package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.PoseUtil
import org.firstinspires.ftc.teamcode.config.util.StateUtil

@Autonomous(group = "Red Auto", name = "Top Flower Fire To Park"/*, preselectTeleOp = "Teleop"*/)
class TopRedFireFlowerToPark : AutoOpMode() {
    override val alliance = Robot.Alliance.RED
    override fun onInit() {
        robot.follower.setPose(PoseUtil.Poses.Red.topRedBlueFlowerSideStart)
    }

    override fun onStart() {
        Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
    }

    fun runAuto(): Command {
        return Groups.sequential(
            Commands.instant { StateUtil.hoodAngle = 0.07; StateUtil.wheelPower = 0.53 },
            Commands.waitMs(1000.0),
            firingSS.letGo(),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topRedBlueFlowerSideToPark()
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
        )
    }
}