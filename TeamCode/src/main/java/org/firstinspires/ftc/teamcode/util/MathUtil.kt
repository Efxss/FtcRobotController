package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs

/** An object made to hold different math related functions for ease of use */
object MathUtil {
    /** A function made to take 2 doubles and check if there are in a margin of error and return true or false */
    fun closeTo(a: Double, b: Double, epsilon: Double = 0.001): Boolean {
        return abs(a - b) < epsilon
    }
}