package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.max
import kotlin.math.min

class DriveUtil(
    hardwareMap: HardwareMap,
    private val drivePower: Double = 0.2,
    private val deadzone: Float = 0.2f
) {

    private val lDrive: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lDrive")
    private val rDrive: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rDrive")

    var velocityModeInitialized = false
    var velocityPowerScale = 1.0

    private val p = 11.0
    private val i = 0.0
    private val d = 0.15
    private val f = 0.035

    init {
        rDrive.direction = DcMotorSimple.Direction.REVERSE
        ensureVelocityMode()
        setPIDF()
    }

    fun tankDrive(leftStick: Float, rightStick: Float) {
        setMotorVelocityFromPseudoPower(lDrive, processInput(leftStick))
        setMotorVelocityFromPseudoPower(rDrive, processInput(rightStick))
    }

    private fun processInput(input: Float): Double {
        return when {
            input > deadzone -> drivePower
            input < -deadzone -> -drivePower
            else -> 0.0
        }
    }

    fun setPIDF() {
        listOf(lDrive, rDrive)
            .forEach { it.setVelocityPIDFCoefficients(p, i, d, f) }
    }

    fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            lDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
            rDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityModeInitialized = true
        }
    }
    fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        motor.velocity = powerToTicksPerSecond(motor, power)
    }
    fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }
    fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }
}