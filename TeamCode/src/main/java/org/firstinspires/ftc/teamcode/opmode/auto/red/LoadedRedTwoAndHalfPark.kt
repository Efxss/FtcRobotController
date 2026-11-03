package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode

@Disabled
@Autonomous(group = "Red Auto", name = "Bottom Cell Two Half Tip Park"/*, preselectTeleOp = "Teleop"*/)
class LoadedRedTwoAndHalfPark : AutoOpMode() {
    override val alliance = Robot.Alliance.RED
    override fun onInit() {
        robot.initPedro()
    }

    override fun onStart() {
        //Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
        robot.follower.update()
    }

    //fun runAuto(): Command {
    //    return Groups.sequential(
    //        Commands.waitMs(1000.0),
    //        firingSS.letGo(),
    //        PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomCellWallOut()),
    //        PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomCellWallOutToRedSpike()),
    //        flowerDSS.down(),
    //        PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.redSpikeToTopScoring()),
    //        firingSS.letGo(),
    //        PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topSpikeScoringToTopFlower()),
    //        PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topFlowerToFlowerScore()),
    //        firingSS.letGo(),
    //        PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.topFlowerScoreToBottomFlower()),
    //        Groups.parallel(
    //            Groups.sequential(
    //                Commands.waitMs(1000.0),
    //                flowerDSS.back()
    //            ),
    //            PedroCommands.follow(robot.follower, PoseUtil.Paths.Red.bottomFlowerToPark())
    //        )
    //    )
    //}
}