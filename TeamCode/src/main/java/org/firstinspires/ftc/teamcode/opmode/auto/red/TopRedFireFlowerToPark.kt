package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.pedropathing.api.Paths.curve
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants

@Autonomous(group = "Red Auto", name = "Top Flower Fire To Park"/*, preselectTeleOp = "Teleop"*/)
class TopRedFireFlowerToPark : AutoOpMode() {
    override val alliance = Robot.Alliance.RED
    lateinit var topRedBlueFlowerSideStart: Pose
    override fun onInit() {
        topRedBlueFlowerSideStart = robot.p.of(58.4, 133.4, 270.0)
        robot.follower.setPose(topRedBlueFlowerSideStart)
    }

    override fun onStart() {
        Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
    }

    fun runAuto(): Command {
        val topRedBlueFlowerSideToPark: Pose = robot.p.of(15.0829, 125.145, 270.0)
        val topRedBlueFlowerSideToParkControl1: Pose = robot.p.of(62.5904, 102.3984, 0.0)
        fun topRedBlueFlowerSideToPark(): Path {
            return curve(
                topRedBlueFlowerSideStart,
                topRedBlueFlowerSideToParkControl1,
                topRedBlueFlowerSideToPark
            ).constant(
                topRedBlueFlowerSideToPark,
            )
        }
        return Groups.sequential(
            Commands.instant { firingSS.hoodAngle = 0.07; firingSS.power = 0.53 },
            Commands.waitMs(1000.0),
            firingSS.execFiring(false),
            PedroCommands.follow(robot.follower, topRedBlueFlowerSideToPark()
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5))),
        )
    }
}