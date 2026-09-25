package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.config.Robot
import java.util.EnumMap

class FiringSS(
    private val robot: Robot,
    private val follower: Follower,
    private val alliance: Robot.Alliance,
    private var fieldHalf: Robot.FieldHalf
) {
    fun execFiring(): Command {
        val turnHalf = EnumMap<Robot.FieldHalf, Command>(Robot.FieldHalf::class.java)
        fun topBlueHive(): Command = PedroCommands.hold(follower,Pose(follower.pose().x(),follower.pose().y(),robot.upFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose))))
        fun bottomBlueHive(): Command = PedroCommands.hold(follower,Pose(follower.pose().x(),follower.pose().y(),robot.downFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose))))
        fun topRedHive(): Command = PedroCommands.hold(follower,Pose(follower.pose().x(),follower.pose().y(),robot.upFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose))))
        fun bottomRedHive(): Command = PedroCommands.hold(follower,Pose(follower.pose().x(),follower.pose().y(),robot.downFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose))))
        when (alliance) {
            Robot.Alliance.BLUE -> {
                turnHalf[Robot.FieldHalf.TOP] = topBlueHive()
                turnHalf[Robot.FieldHalf.BOTTOM] = bottomBlueHive()
            }
            Robot.Alliance.RED -> {
                turnHalf[Robot.FieldHalf.TOP] = topRedHive()
                turnHalf[Robot.FieldHalf.BOTTOM] = bottomRedHive()
            }
        }
        fun turnHalfFun(): Command = Commands.match({fieldHalf}, turnHalf)
        fun letGo(): Command {
            fun out(): Command = Commands.instant { robot.pollenBS.set(1.0) }
            fun keep(): Command = Commands.instant { robot.pollenBS.set(0.0) }
            return Groups.sequential(
                out(),
                Commands.waitMs(500.0),
                keep()
            )
        }
        return sequential(
            //turnTo(robot.follower, Math.toRadians(robot.downFireRedTab.get(robot.follower.pose().distance(robot.refPose))))
            turnHalfFun(),
            letGo()
        )
    }
    fun calcFiring(): Command {
        return Commands.infinite {
            when (alliance) {
                Robot.Alliance.BLUE -> {
                    if (fieldHalf == Robot.FieldHalf.TOP) robot.fireM.set(robot.upFireBlueTab.get(follower.pose().distance(robot.refPose)))
                    else robot.fireM.set(robot.downFireBlueTab.get(follower.pose().distance(robot.refPose)))
                }
                Robot.Alliance.RED -> {
                    if (fieldHalf == Robot.FieldHalf.TOP) robot.fireM.set(robot.upFireRedTab.get(follower.pose().distance(robot.refPose)))
                    else robot.fireM.set(robot.downFireRedTab.get(follower.pose().distance(robot.refPose)))
                }
            }
        }
    }
    fun calcHalf(): Command = Commands.infinite { fieldHalf = if (follower.pose().y() >= 72) Robot.FieldHalf.TOP else Robot.FieldHalf.BOTTOM }
}