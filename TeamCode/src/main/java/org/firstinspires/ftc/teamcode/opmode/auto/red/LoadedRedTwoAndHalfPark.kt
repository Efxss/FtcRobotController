package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.interpolator.Interpolator
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.StateUtil

@Autonomous(group = "Red Auto", name = "Bottom Cell Two Half Tip Park"/*, preselectTeleOp = "Teleop"*/)
class LoadedRedTwoAndHalfPark : AutoOpMode() {
    override val alliance = Robot.Alliance.RED
    val bottomCellStartWall: Pose = robot.p.of(55.0, 8.0, 90.0)
    override fun onInit() {
        robot.follower.setPose(bottomCellStartWall)
    }

    override fun onStart() {
        Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
    }

    fun runAuto(): Command {
        val bottomCellPullOut: Pose = robot.p.of(55.0, 16.0, 90.0)
        val spike: Pose = robot.p.of(7.0, 7.5, 180.0)
        val spikeControl1: Pose = robot.p.of(65.0567, 20.7167, 0.0)
        val topCellSpikeScoring: Pose = robot.p.of(26.8084, 119.3781, -50.0)
        val topFlower: Pose = robot.p.of(36.9637, 129.4173, 47.0)
        val topFlowerScore: Pose = robot.p.of(33.5295, 126.318, -50.0)
        val bottomFlower: Pose = robot.p.of(16.9047, 46.6075, 180.0)
        val bottomFlowerToBottomCell: Pose = robot.p.of(55.0, 8.0, 90.0)
        val bottomFlowerToBottomCellSegment1Heading: Pose = robot.p.of(55.0, 8.0, 180.0)
        val bottomFlowerToBottomCellSegment2Heading: Pose = robot.p.of(55.0, 8.0, 90.0)
        val bottomCellFireToPark: Pose = robot.p.of(10.4154, 87.9747, 90.0)
        fun bottomCellWallOutFire(): Path {
            return line(
                bottomCellStartWall,
                bottomCellPullOut
            )
            .constant(
                bottomCellPullOut
            )
        }
        fun bottomCellOutToRedSpike(): Path {
            return line(
                bottomCellPullOut,
                spike
            )
            .constant(
                spike
            )
        }
        fun redSpikeToTopScoring(): Path {
            return line(
                spike,
                topCellSpikeScoring
            )
            .heading(
                Interpolator.piecewise()
                    .until(0.8, Interpolator.constant(spike))
                    .until(1.0, Interpolator.constant(topCellSpikeScoring))
            )
        }
        fun topSpikeScoringToTopFlower(): Path {
            return line(
                topCellSpikeScoring,
                topFlower
            )
            .constant(
                topFlower
            )
        }
        fun topFlowerToFlowerScore(): Path {
            return line(
                topFlower,
                topFlowerScore
            )
            .linear(
                topFlower,
                topFlowerScore
            )
        }
        fun topFlowerScoreToBottomFlower(): Path {
            return line(
                topFlowerScore,
                bottomFlower
            )
            .constant(
                bottomFlower
            )
        }
        fun bottomFlowerToBottomCell(): Path {
            return line(
                bottomFlower,
                bottomFlowerToBottomCell
            )
            .heading(
                Interpolator.piecewise()
                    .until(0.5, Interpolator.constant(180.0))
                    .until(1.0, Interpolator.constant(90.0))
            )
        }
        fun bottomCellFireToPark(): Path {
            return line(
                bottomFlowerToBottomCell,
                bottomCellFireToPark
            )
            .constant(
                bottomCellFireToPark
            )
        }
        return Groups.sequential(
            Commands.instant { firingSS.hoodAngle = 0.07; firingSS.power = 0.53 },
            Commands.waitMs(1000.0),
            PedroCommands.follow(robot.follower, bottomCellWallOutFire()),
            firingSS.letGo(),
            PedroCommands.follow(robot.follower, bottomCellOutToRedSpike()
                .with(Constants.foresightConfig.maxPathSpeed.at(0.9))),
            Groups.parallel(
                flowerDSS.down(),
                PedroCommands.follow(robot.follower, redSpikeToTopScoring()),
            ),
            PedroCommands.follow(robot.follower, redSpikeToTopScoring()),
            firingSS.letGo(),
            Groups.deadline(
                intakeSS.isFullDeadline(),
                PedroCommands.follow(robot.follower, topSpikeScoringToTopFlower()),
            ),
            PedroCommands.follow(robot.follower, topFlowerToFlowerScore()),
            firingSS.letGo(),
            Groups.deadline(
                intakeSS.isFullDeadline(),
                PedroCommands.follow(robot.follower, topFlowerScoreToBottomFlower()),
            ),
            Groups.parallel(
                Groups.sequential(
                    Commands.waitMs(700.0),
                    flowerDSS.back()
                ),
                PedroCommands.follow(robot.follower, bottomFlowerToBottomCell()),
            ),
            PedroCommands.follow(robot.follower, bottomCellFireToPark())
        )
    }
}