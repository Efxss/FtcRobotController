package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.util.PoseUtil

@Disabled
@Autonomous(group = "Red Auto", name = "Bottom Cell Two Half Tip Park"/*, preselectTeleOp = "Teleop"*/)
class LoadedRedTwoAndHalfPark : AutoOpMode() {
    override val alliance = Robot.Alliance.RED
    override fun onInit() {
        robot.initPedro()
    }

    override fun onStart() {
        Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
        robot.follower.update()
    }

    fun runAuto(): Command {
        return Groups.sequential(
            Commands.waitMs(1000.0),
            firingSS.letGo(),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomCellWallToRedSpike()),
            //Groups.parallel(
            //    flowerDSS.down(),
            //    PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.redSpikeToTopScoring()),
            //),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.redSpikeToTopScoring()),
            firingSS.letGo(),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topSpikeScoringToTopFlower()),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topFlowerToFlowerScore()),
            firingSS.letGo(),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topFlowerScoreToBottomFlower()),
            //Groups.parallel(
            //    Groups.sequential(
            //        Commands.waitMs(700.0),
            //        flowerDSS.back()
            //    ),
            //    PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomFlowerToPark())
            //)
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomFlowerToBottomCell()),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomCellFireToPark())
        )
    }
}