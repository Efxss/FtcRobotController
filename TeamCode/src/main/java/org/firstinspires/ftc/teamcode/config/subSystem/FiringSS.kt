package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups.sequential
//import com.pedropathing.ivy.pedro.PedroCommands
//import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.config.Robot
//import java.util.EnumMap

class FiringSS(
    private val robot: Robot,
    private val follower: Follower,
    private val alliance: Robot.Alliance,
) {
    private val servoBlock = 0.0
    private val servoGo = 0.33
    private var fieldHalf: Robot.FieldHalf = Robot.FieldHalf.TOP
    fun letGo(): Command {
        return sequential(
            Commands.instant { robot.pollenBS.set(servoGo) },
            //Commands.waitMs(1250.0),
            Commands.waitMs(10000.0),
            Commands.instant { robot.pollenBS.set(servoBlock) }
        ).requiring(robot.pollenBS)
    }
    fun execFiring(): Command {
        //val turnHalf = EnumMap<Robot.FieldHalf, Command>(Robot.FieldHalf::class.java)
        //fun topBlueCell(): Command = PedroCommands.hold(follower,Pose(follower.pose().x(),follower.pose().y(),Math.toRadians(robot.upFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose)))))
        //fun bottomBlueCell(): Command = PedroCommands.hold(follower, Pose(follower.pose().x(),follower.pose().y(),Math.toRadians(robot.downFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose)))))
        //fun topRedCell(): Command = PedroCommands.hold(follower,Pose(follower.pose().x(),follower.pose().y(),Math.toRadians(robot.upFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose)))))
        //fun bottomRedCell(): Command = PedroCommands.hold(follower,Pose(follower.pose().x(),follower.pose().y(),Math.toRadians(robot.downFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose)))))
        //when (alliance) {
        //    Robot.Alliance.BLUE -> {
        //        turnHalf[Robot.FieldHalf.TOP] = topBlueCell()
        //        turnHalf[Robot.FieldHalf.BOTTOM] = bottomBlueCell()
        //    }
        //    Robot.Alliance.RED -> {
        //        turnHalf[Robot.FieldHalf.TOP] = topRedCell()
        //        turnHalf[Robot.FieldHalf.BOTTOM] = bottomRedCell()
        //    }
        //}
        //fun turnHalfFun(): Command = Commands.match({fieldHalf}, turnHalf)
        return sequential(
            //turnHalfFun(),
            letGo()
        )
    }
    //fun calcFiring(): Command {
    //    return Commands.infinite {
    //        when (alliance) {
    //            Robot.Alliance.BLUE -> {
    //                if (fieldHalf == Robot.FieldHalf.TOP) {
    //                    robot.fireMSquIDF.setPoint = robot.upFireBlueTab.get(follower.pose().distance(robot.refPose))
    //                    robot.fireM.set(robot.fireMSquIDF.calculate())
    //                }
    //                else {
    //                    robot.fireMSquIDF.setPoint = robot.downFireBlueTab.get(follower.pose().distance(robot.refPose))
    //                    robot.fireM.set(robot.fireMSquIDF.calculate())
    //                }
    //            }
    //            Robot.Alliance.RED -> {
    //                if (fieldHalf == Robot.FieldHalf.TOP) {
    //                    robot.fireMSquIDF.setPoint = robot.upFireRedTab.get(follower.pose().distance(robot.refPose))
    //                    robot.fireM.set(robot.fireMSquIDF.calculate())
    //                }
    //                else {
    //                    robot.fireMSquIDF.setPoint = robot.downFireRedTab.get(follower.pose().distance(robot.refPose))
    //                    robot.fireM.set(robot.fireMSquIDF.calculate())
    //                }
    //            }
    //        }
    //    }
    //}
    fun calcHalf(): Command = Commands.infinite { fieldHalf = if (follower.pose().y() >= 72) Robot.FieldHalf.TOP else Robot.FieldHalf.BOTTOM }
    fun getHalf(): Robot.FieldHalf = fieldHalf
    fun reset() {
        robot.pollenBS.set(servoBlock)
    }
}