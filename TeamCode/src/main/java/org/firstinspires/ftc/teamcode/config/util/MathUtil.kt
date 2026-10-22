package org.firstinspires.ftc.teamcode.config.util

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
     * @author NOTE : THIS WAS MADE FOR A 6000 RPM MOTOR MIGHT NOT WORK ON OTHER MOTORS
     */
    fun pidfPower(input: Double): Double {
        return input * 2800
    }
}
