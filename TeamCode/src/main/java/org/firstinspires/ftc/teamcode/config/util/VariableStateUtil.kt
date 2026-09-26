package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.config.Robot

/**
 * Holds state that needs to persist between OpModes.
 * Because this is a Kotlin `object`, it lives in the JVM for the entire so
 * It survives the OpMode lifecycles
 */
object VariableStateUtil {
    // Null means "no auto ran" -> TeleOp should fall back to a default pose.
    var endOfAutoPose: Pose = Pose(0.0,0.0, 0.0)

    // Alliance var to pass from auto to TeleOP
    var alliance: Robot.Alliance = Robot.Alliance.BLUE
}