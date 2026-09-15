package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.groups.Groups
import org.firstinspires.ftc.teamcode.config.Robot

class FiringSS(
    robot: Robot
) {
    val execFiring: Command = Groups.sequential(

    )
    fun calcFiring(robot: Robot, alliance: Robot.Alliance, follower: Follower) {
        when (alliance) {
            Robot.Alliance.BLUE -> {
                if (follower.pose.y >= 72) {
                    robot.fireM.set(robot.upFireBlueTab.get(follower.pose.distanceFrom(Pose(0.0, 0.0))))
                }
            }
            else -> {
                if (follower.pose.y >= 72) {
                    robot.fireM.set(robot.upFireBlueTab.get(follower.pose.distanceFrom(Pose(0.0, 0.0))))
                }
            }
        }
    }
}