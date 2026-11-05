package org.firstinspires.ftc.teamcode.opmode.auto.blue

import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants

@Disabled
@Autonomous(group = "Auto", name = "Test Auto"/*, preselectTeleOp = "Teleop"*/)
class BlueAuto : AutoOpMode() {
    override val alliance = Robot.Alliance.BLUE

    override fun onInit() {
        robot.initPedro()
        robot.follower.setPose(Pose(106.0, 8.0, 0.0))
        robot.follower.update()
    }

    override fun onStart() {
        Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
    }
    fun runAuto(): Command {
        fun test(): Path {
            return line(
                Pose(106.0, 8.0, 0.0),
                Pose(137.0, 7.5, 0.0),
            )
            .constant(
                0.0
            )
        }
        return Groups.sequential(
         PedroCommands.follow(robot.follower, test()
             .with(Constants.foresightConfig.maxPathSpeed.at(0.5)))
        )
    }
}