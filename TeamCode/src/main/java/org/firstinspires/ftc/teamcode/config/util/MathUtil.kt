package org.firstinspires.ftc.teamcode.config.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign

/** An object made to hold different math related functions for ease of use */
object MathUtil {
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

    /**
     * Applies a deadzone and an exponential power curve to a stick input for finer low-end control while still reaching full power at full deflection
     * @param [input] Raw stick value expected in [-1.0, 1.0]
     * @param [deadzone] Ignores anything under this value to account for stick noise
     * @param [exponent] Curve steepness 1.0 is linear 2.0 is quadratic (recommended) higher gives gentler low end
     * @param [maxPower] Maximum output magnitude useful for implementing a slow mode
     *
     */
    @Deprecated("Unused right now might need to remove from code") fun shapeStick(input : Double, deadzone : Double = 0.01, exponent : Double = 2.0, maxPower : Double = 1.0) : Double {
        if (abs(input) < deadzone) return 0.0
        val scaled = (abs(input) - deadzone) / (1.0 - deadzone)
        return sign(input) * scaled.pow(exponent) * maxPower
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
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf)
    }

    fun clip(v : Double, minValue : Double, maxValue : Double) : Double {
        return max(minValue, min(maxValue, v))
    }
}
