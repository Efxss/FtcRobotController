package org.firstinspires.ftc.teamcode.opmode.auto.blue

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.util.PoseUtil

@Autonomous(group = "Auto", name = "Test Auto")
class BlueAuto : AutoOpMode() {
    override val alliance = Robot.Alliance.BLUE

    override fun onInit() {
        robot.initPedro()
        robot.follower.setPose(PoseUtil.Poses.Blue.startPoseTest)
        robot.follower.update()
    }

    override fun onStart() {
        //intakeSS.runIntake().schedule()
        Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
        robot.follower.update()
    }
    fun runAuto(): Command = Groups.sequential(
        PedroCommands.follow(robot.follower, PoseUtil.Paths.Blue.testPath()),
        Commands.waitMs(1000.0),
        Groups.repeat(spin(), 5)
    )
    fun spin(): Command {
        fun first(): Command {
            return PedroCommands.hold(robot.follower, PoseUtil.Poses.Blue.movePoseTest)
        }
        fun next(): Command {
            return PedroCommands.hold(robot.follower, PoseUtil.Poses.Blue.movePoseTestHead)
        }
        return Groups.sequential(
            first(),
            Commands.waitMs(2500.0),
            next(),
            Commands.waitMs(2500.0),
        )
    }
}