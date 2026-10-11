package org.firstinspires.ftc.teamcode.config.subSystem

//import java.util.EnumMap
//import com.pedropathing.ivy.pedro.PedroCommands
//import com.pedropathing.math.Pose
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.groups.Groups.sequential
import org.firstinspires.ftc.teamcode.config.Robot

class FiringSS(
    private val robot: Robot,
    private val follower: Follower,
    private val alliance: Robot.Alliance,
    private val intakeSS: IntakeSS,
    private val servoBlock: Double = 0.02,
    private val servoGo: Double = 0.4
) {
    private var fieldHalf: Robot.FieldHalf = Robot.FieldHalf.TOP
    fun letGo(): Command {
        return sequential(
            Commands.instant { robot.pollenBS.set(servoGo) },
            //Commands.waitMs(1250.0),
            Commands.waitMs(2500.0),
            Commands.instant { robot.pollenBS.set(servoBlock) }
        ).requiring(robot.pollenBS)
    }
    fun execFiring(): Command {
        //val turnHalf = EnumMap<Robot.FieldHalf, Command>(Robot.FieldHalf::class.java)
        //val fastFireRange = 0.0..30.0
        //val fastFire = 1.0
        //val medFireRange = 30.0..60.0
        //val medFire = 0.9
        //val slowFireRange = 60.0..90.0
        //val slowFire = 0.8
        //var remain: Double
        //fun topBlueCell(): Command {
        //    fun intakeSpd(): Command {
        //        return Commands.instant {
        //            remain = robot.upFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose)) - robot.follower.pose().heading()
        //            when (abs(remain)) {
        //                in fastFireRange -> intakeSS.velocity = fastFire
        //                in medFireRange -> intakeSS.velocity = medFire
        //                in slowFireRange -> intakeSS.velocity = slowFire
        //                else -> intakeSS.velocity = robot.intakeDef
        //            }
        //        }
        //    }
        //    fun turn(): Command {
        //        return PedroCommands.hold(follower,Pose(
        //            follower.pose().x(),
        //            follower.pose().y(),
        //            Math.toRadians(robot.upFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose))))
        //        )
        //    }
        //    return Groups.parallel(intakeSpd(),turn())
        //}
        //fun bottomBlueCell(): Command {
        //    fun intakeSpd(): Command {
        //        return Commands.instant {
        //            remain = robot.downFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose)) - robot.follower.pose().heading()
        //            when (abs(remain)) {
        //                in fastFireRange -> intakeSS.velocity = fastFire
        //                in medFireRange -> intakeSS.velocity = medFire
        //                in slowFireRange -> intakeSS.velocity = slowFire
        //                else -> intakeSS.velocity = robot.intakeDef
        //            }
        //        }
        //    }
        //    fun turn(): Command {
        //        return PedroCommands.hold(follower, Pose(
        //            follower.pose().x(),
        //            follower.pose().y(),
        //            Math.toRadians(robot.downFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose))))
        //        )
        //    }
        //    return Groups.parallel(intakeSpd(),turn())
        //}
        //fun topRedCell(): Command {
        //    fun intakeSpd(): Command {
        //        return Commands.instant {
        //            remain = robot.upFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose)) - robot.follower.pose().heading()
        //            when (abs(remain)) {
        //                in fastFireRange -> intakeSS.velocity = fastFire
        //                in medFireRange -> intakeSS.velocity = medFire
        //                in slowFireRange -> intakeSS.velocity = slowFire
        //                else -> intakeSS.velocity = robot.intakeDef
        //            }
        //        }
        //    }
        //    fun turn(): Command {
        //        return PedroCommands.hold(follower,Pose(
        //            follower.pose().x(),
        //            follower.pose().y(),
        //            Math.toRadians(robot.upFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose))))
        //        )
        //    }
        //    return Groups.parallel(intakeSpd(),turn())
        //}
        //fun bottomRedCell(): Command {
        //    fun intakeSpd(): Command {
        //        return Commands.instant {
        //            remain = robot.downFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose)) - robot.follower.pose().heading()
        //            when (abs(remain)) {
        //                in fastFireRange -> intakeSS.velocity = fastFire
        //                in medFireRange -> intakeSS.velocity = medFire
        //                in slowFireRange -> intakeSS.velocity = slowFire
        //                else -> intakeSS.velocity = robot.intakeDef
        //            }
        //        }
        //    }
        //    fun turn(): Command {
        //        return PedroCommands.hold(follower,Pose(
        //            follower.pose().x(),
        //            follower.pose().y(),
        //            Math.toRadians(robot.downFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose))))
        //        )
        //    }
        //    return Groups.parallel(intakeSpd(),turn())
        //}
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
        return Groups.parallel(
            letGo(),
            //turnHalfFun(),
            //Groups.sequential(
            //    Commands.waitMs(5000.0),
            //    Commands.instant { intakeSS.velocity = robot.intakeDef }
            //)
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
    //fun calcHalf(): Command = Commands.infinite { fieldHalf = if (follower.pose().y() >= 72) Robot.FieldHalf.TOP else Robot.FieldHalf.BOTTOM }
    fun getHalf(): Robot.FieldHalf = fieldHalf
    fun reset() {
        robot.pollenBS.set(servoBlock)
    }
}