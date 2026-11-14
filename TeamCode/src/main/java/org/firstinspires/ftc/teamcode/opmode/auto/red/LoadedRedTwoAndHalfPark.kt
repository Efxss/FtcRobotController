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

@Autonomous(group = "Red Auto", name = "Bottom Cell Two Half Tip Park"/*, preselectTeleOp = "Teleop"*/)
class LoadedRedTwoAndHalfPark : AutoOpMode() {
    override val alliance = Robot.Alliance.RED
    override fun onInit() {
        robot.follower.setPose(PoseUtil.Poses.Red.bottomCellStartWall)
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
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomCellWallOutFire()),
            firingSS.letGo(),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomCellOutToRedSpike()
                .with(Constants.foresightConfig.maxPathSpeed.at(0.9))),
            Groups.parallel(
                flowerDSS.down(),
                PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.redSpikeToTopScoring()),
            ),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.redSpikeToTopScoring()),
            firingSS.letGo(),
            Groups.deadline(
                intakeSS.isFullDeadline(),
                PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topSpikeScoringToTopFlower()),
            ),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topFlowerToFlowerScore()),
            firingSS.letGo(),
            Groups.deadline(
                intakeSS.isFullDeadline(),
                PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topFlowerScoreToBottomFlower()),
            ),
            Groups.parallel(
                Groups.sequential(
                    Commands.waitMs(700.0),
                    flowerDSS.back()
                ),
                PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomFlowerToBottomCell()),
            ),
            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomCellFireToPark())
        )
    }
}