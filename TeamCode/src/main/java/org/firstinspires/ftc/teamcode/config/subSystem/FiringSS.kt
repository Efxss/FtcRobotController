package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands.turnTo
import org.firstinspires.ftc.teamcode.config.Robot

class FiringSS(
    robot: Robot
) {
    fun execFiring(robot: Robot, alliance: Robot.Alliance): Command {
        return Groups.sequential(
            turnTo(robot.follower, robot.downFireRedTab.get(robot.follower.pose.distanceFrom(robot.refPose)))
    )}
    fun calcFiring(robot: Robot, alliance: Robot.Alliance, follower: Follower) {
        when (alliance) {
            Robot.Alliance.BLUE -> {
                if (follower.pose.y >= 72) {
                    robot.fireM.set(robot.upFireBlueTab.get(follower.pose.distanceFrom(robot.refPose)))
                } else {
                    robot.fireM.set(robot.downFireBlueTab.get(follower.pose.distanceFrom(robot.refPose)))
                }
            }
            Robot.Alliance.RED -> {
                if (follower.pose.y >= 72) {
                    robot.fireM.set(robot.upFireRedTab.get(follower.pose.distanceFrom(robot.refPose)))
                } else {
                    robot.fireM.set(robot.downFireRedTab.get(follower.pose.distanceFrom(robot.refPose)))
                }
            }
        }
    }
}