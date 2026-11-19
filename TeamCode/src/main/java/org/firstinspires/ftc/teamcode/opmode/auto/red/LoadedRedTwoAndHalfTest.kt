package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.pedropathing.api.Paths
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

@Autonomous(group = "Red Auto", name = "Bottom Cell Two Half Tip Park Test"/*, preselectTeleOp = "Teleop"*/)
class LoadedRedTwoAndHalfTest : AutoOpMode() {
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
        val bottomFlower: Pose = robot.p.of(12.0, 47.0, 180.0)

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
        return Groups.sequential(
            Commands.instant { firingSS.hoodAngle = 0.09; firingSS.power = 0.9 },
            Commands.waitMs(1000.0),
            PedroCommands.follow(robot.follower, bottomCellPullOut()),
            firingSS.letGo(),
            Groups.parallel(
                PedroCommands.follow(robot.follower, bottomFlowerOut()),
                flowerDSS.down()
            ),
            PedroCommands.follow(robot.follower, bottomFlower()
                .with(Constants.foresightConfig.maxPathSpeed.at(0.6))
            ),
            Commands.waitMs(1000.0),
            Groups.sequential(
                PedroCommands.follow(robot.follower, bottomFlowerRepeatOut()
                    .with(Constants.foresightConfig.maxPathSpeed.at(0.4))
                ),
                PedroCommands.follow(robot.follower, bottomFlowerRepeatIn()
                    .with(Constants.foresightConfig.maxPathSpeed.at(0.4))
                ),
            ),
        )
    }
}