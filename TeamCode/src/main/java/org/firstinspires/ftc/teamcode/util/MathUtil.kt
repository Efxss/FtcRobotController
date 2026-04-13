package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/** An object made to hold different math related functions for ease of use */
object MathUtil {
    var isPIDFAply = false
    /** A function made to take 2 doubles and check if there are in a margin of error and return true or false */
    fun closeTo(a : Double, b : Double, epsilon : Double = 0.001) : Boolean {
        return abs(a - b) < epsilon
    }

    /**
     * Calling this function will allow you to convert a power to velocity for more precise control with things such as PIDF controllers
     * @param [velocityPowerScale] Is a scale at which it will convert the power to velocity for example if a specific power works for a flywheel but the conversion makes it too much you can drop the scale to convert less of the power
     */
    fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double, velocityPowerScale : Double, pidf : PIDFCoefficients) {
        motor.velocity = powerToTicksPerSecond(motor, power, velocityPowerScale, pidf)
    }

    fun powerToTicksPerSecond(motor : DcMotorEx, power : Double, velocityPowerScale : Double, pidf : PIDFCoefficients) : Double {
        applyPIDF(pidf, motor)
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }

    fun applyPIDF(pidf : PIDFCoefficients, motor : DcMotorEx) {
        fun applyPIDF(pidf: PIDFCoefficients, motor: DcMotorEx) {
            if (!isPIDFAply) {
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf)
                isPIDFAply = true
            }
        }
    }

    fun clip(v : Double, minValue : Double, maxValue : Double) : Double {
        return max(minValue, min(maxValue, v))
    }
}
