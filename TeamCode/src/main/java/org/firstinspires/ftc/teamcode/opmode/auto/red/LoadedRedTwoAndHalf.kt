package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.pedropathing.api.Paths
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

@Autonomous(group = "Red Auto", name = "Bottom Cell Two Half Tip Park"/*, preselectTeleOp = "Teleop"*/)
class LoadedRedTwoAndHalf : AutoOpMode() {
    override val alliance = Robot.Alliance.RED
    lateinit var bottomCellStartWall: Pose
    override fun onInit() {
        bottomCellStartWall = robot.p.of(55.0, 8.0, 90.0)
        robot.follower.setPose(bottomCellStartWall)
    }

    override fun onStart() {
        Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
    }

    fun runAuto(): Command {
        val bottomCellPullOut: Pose = robot.p.of(55.0, 16.0, 90.0)
        val bottomFlowerOut: Pose = robot.p.of(28.0, 47.0, 180.0)
        val bottomFlower: Pose = robot.p.of(11.2, 47.0, 180.0)
        val bottomFlowerScore: Pose = robot.p.of(34.0233, 121.2689, -70.0)
        val bottomFlowerScoreControl1: Pose = robot.p.of(33.5291, 42.6524, 0.0)
        val bottomFlowerScoreSegment1Start: Pose = robot.p.of(34.0233, 121.2689, 180.0)
        val bottomFlowerScoreSegment1End: Pose = robot.p.of(34.0233, 121.2689, 90.0)
        val bottomFlowerScoreSegment2Heading: Pose = robot.p.of(34.0233, 121.2689, 90.0)
        val bottomFlowerScoreSegment3Heading: Pose = robot.p.of(34.0233, 121.2689, -70.0)
        val topFlower: Pose = robot.p.of(46.6, 126.0, 90.0)
        val topFlowerControl1: Pose = robot.p.of(46.9796, 120.3153, 0.0)
        val topFlowerOut: Pose = robot.p.of(46.6, 120.0, 90.0)
        val topFlowerScore: Pose = robot.p.of(46.6, 118.0, -80.0)
        val topFlowerScoreSegment1Start: Pose = robot.p.of(46.6, 118.0, 90.0)
        val topFlowerScoreSegment1End: Pose = robot.p.of(46.6, 118.0, -80.0)
        val topFlowerScoreSegment2Heading: Pose = robot.p.of(46.6, 118.0, -80.0)

        fun bottomCellPullOut(): Path {
          return Paths.line(
              bottomCellStartWall,
              bottomCellPullOut
          ).constant(
              bottomCellPullOut
          )
        }
        fun bottomFlowerOut(): Path {
            return Paths.line(
                bottomCellPullOut,
                bottomFlowerOut
            )
            .constant(
                bottomFlowerOut
            )
        }
        fun bottomFlower(): Path {
            return Paths.line(
                bottomFlowerOut,
                bottomFlower
            ).constant(
                bottomFlower
            )
        }
        fun bottomFlowerRepeatOut(): Path {
            return Paths.line(
                bottomFlower,
                bottomFlowerOut
            ).constant(
                bottomFlower
            )
        }
        fun bottomFlowerRepeatIn(): Path {
            return Paths.line(
                bottomFlowerOut,
                bottomFlower
            ).constant(
                bottomFlower
            )
        }
        fun bottomFlowerScore(): Path {
            return Paths.curve(
                bottomFlower,
                bottomFlowerScoreControl1,
                bottomFlowerScore
            ).heading(
            Interpolator.piecewise()
                .until(
                    0.25,
                    Interpolator.linear(
                        bottomFlowerScoreSegment1Start,
                        bottomFlowerScoreSegment1End
                    )
                )
                .until(
                    0.99,
                    Interpolator.constant(bottomFlowerScoreSegment2Heading)
                )
                .until(
                    1.0,
                    Interpolator.constant(bottomFlowerScoreSegment3Heading)
                )
            )
        }
        fun topFlower(): Path {
            return Paths.curve(
                bottomFlowerScore,
                topFlowerControl1,
                topFlower
            ).constant(
            topFlower
            )
        }
        fun topFlowerRepeatOut(): Path {
            return Paths.line(
                topFlower,
                topFlowerOut
            ).constant(
                topFlower
            )
        }
        fun topFlowerRepeatIn(): Path {
            return Paths.line(
                topFlowerOut,
                topFlower
            ).constant(
                topFlower
            )
        }
        fun topFlowerScore(): Path {
            return Paths.line(
                topFlowerOut,
                topFlowerScore
            ).heading(
            Interpolator.piecewise()
                .until(
                    0.9,
                    Interpolator.linear(
                        topFlowerScoreSegment1Start,
                        topFlowerScoreSegment1End
                    )
                )
                .until(
                    1.0,
                    Interpolator.constant(topFlowerScoreSegment2Heading)
                )
            )
        }
        return Groups.sequential(
            Groups.deadline(
                Commands.waitMs(900.0),
                Commands.instant { firingSS.hoodAngle = 0.04; firingSS.power = 0.7 },
                PedroCommands.follow(robot.follower, bottomCellPullOut()),
            ),
            firingSS.letGo(),
            Groups.parallel(
                PedroCommands.follow(robot.follower, bottomFlowerOut()),
                flowerDSS.down()
            ),
            PedroCommands.follow(robot.follower, bottomFlower()
                .with(Constants.foresightConfig.maxPathSpeed.at(0.65))
            ),
            Commands.waitMs(700.0),
            Groups.sequential(
                PedroCommands.follow(robot.follower, bottomFlowerRepeatOut()
                    .with(Constants.foresightConfig.maxPathSpeed.at(0.4))
                ),
                PedroCommands.follow(robot.follower, bottomFlowerRepeatIn()
                    .with(Constants.foresightConfig.maxPathSpeed.at(0.5))
                ),
            ),
            Commands.waitMs(200.0),
            Groups.parallel(
                PedroCommands.follow(robot.follower, bottomFlowerScore()),
                Groups.sequential(
                    Commands.waitMs(1000.0),
                    Commands.instant { intakeSS.speed = 0.1 },
                    flowerDSS.back()
                ),
            ),
            Commands.instant { firingSS.hoodAngle = 0.07; firingSS.power = 0.7; intakeSS.speed = robot.intakeFire },
            Commands.waitMs(800.0),
            firingSS.letGo(),
            Commands.instant { intakeSS.speed = robot.intakeDef },
            Groups.parallel(
                PedroCommands.follow(robot.follower, topFlower()
                    .with(Constants.foresightConfig.maxPathSpeed.at(0.65))),
                flowerDSS.down()
            ),
            Commands.waitMs(700.0),
            Groups.sequential(
                PedroCommands.follow(robot.follower, topFlowerRepeatOut()
                    .with(Constants.foresightConfig.maxPathSpeed.at(0.4))
                ),
                PedroCommands.follow(robot.follower, topFlowerRepeatIn()
                    .with(Constants.foresightConfig.maxPathSpeed.at(0.5))
                ),
            ),
            Commands.waitMs(200.0),
            PedroCommands.follow(robot.follower, topFlowerScore()),
            Commands.instant { firingSS.hoodAngle = 0.02; firingSS.power = 0.7; intakeSS.speed = robot.intakeFire },
            Commands.waitMs(800.0),
            firingSS.letGo(),
            Commands.instant { intakeSS.speed = robot.intakeDef },
        )
    }
}