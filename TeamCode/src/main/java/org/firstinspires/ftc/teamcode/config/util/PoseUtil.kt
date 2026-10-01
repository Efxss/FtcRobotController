package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.api.Paths.line
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import org.firstinspires.ftc.teamcode.config.util.PoseUtil.Poses.Blue.bottomBlueFlower
import org.firstinspires.ftc.teamcode.config.util.PoseUtil.Poses.Blue.movePoseTest
import org.firstinspires.ftc.teamcode.config.util.PoseUtil.Poses.Blue.startPoseTest

object PoseUtil {
    val p: PoseFactory = PoseFactory.degrees()
    object Poses {
        object Blue {
            val startPoseTest: Pose = p.of(9.1, 9.2, 90.0)
            val movePoseTest: Pose = p.of(80.0, 92.0, 90.0)
            val movePoseTestHead: Pose = p.of(80.0, 92.0, 270.0)
            val bottomBlueFlower: Pose = p.of(95.5, 32.0, 270.0)
        }
        object Red {

        }
    }
    object Paths {
        object Blue {
            fun testPath(): Path = line(startPoseTest, movePoseTest).linear(startPoseTest,movePoseTest)
            fun testPathToFlower(): Path = line(movePoseTest, bottomBlueFlower).constant(movePoseTest)
        }
        object Red {

        }
    }
}