package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs

object MathUtil {
    fun closeTo(a: Double, b: Double, epsilon: Double = 0.001): Boolean {
        return abs(a - b) < epsilon
    }
}