package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.config.Robot
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/** An object made to hold different math related functions for ease of use */
object MathUtil {
    /** A function made to take 2 doubles and check if there are in a margin of error and return true or false */
    fun closeTo(a: Double, b: Double, epsilon: Double = 0.001): Boolean {
        return abs(a - b) < epsilon
    }

    fun clip(v: Double, minValue: Double, maxValue: Double): Double {
        return max(minValue, min(maxValue, v))
    }

    /**
     * Returns The Power For A PIDF Controller To Use
     * @author NOTE : THIS WAS MADE FOR A 6000 RPM MOTOR WILL NOT WORK WITH OTHER MOTORS
     */
    fun pidfPower(input: Double): Double {
        return input * 2800
    }

    /**
     * @return THIS WILL RETURN A RADIAN TO NOT CONVERT INTO A RADIAN AGAIN IT IS ALREADY A RADIAN
     */
    fun cellRadian(currentPose: Pose, fieldHalf: Robot.FieldHalf, alliance: Robot.Alliance, robot: Robot): Double {
        var dpx: Double
        var dpy: Double
        when (alliance) {
            Robot.Alliance.BLUE -> {
                when (fieldHalf) {
                    Robot.FieldHalf.TOP -> {
                        dpx = robot.topBlueRefPose.x()
                        dpy = robot.topBlueRefPose.y()
                    }
                    Robot.FieldHalf.BOTTOM -> {
                        dpx = robot.bottomBlueRefPose.x()
                        dpy = robot.bottomBlueRefPose.y()
                    }
                }
            }
            Robot.Alliance.RED -> {
                when (fieldHalf) {
                    Robot.FieldHalf.TOP -> {
                        dpx = robot.topRedRefPose.x()
                        dpy = robot.topRedRefPose.y()
                    }
                    Robot.FieldHalf.BOTTOM -> {
                        dpx = robot.bottomRedRefPose.x()
                        dpy = robot.bottomRedRefPose.y()
                    }
                }
            }
        }
        val dx = dpx - currentPose.x()
        val dy = dpy - currentPose.y()
        return kotlin.math.atan2(dy, dx)
    }
}
