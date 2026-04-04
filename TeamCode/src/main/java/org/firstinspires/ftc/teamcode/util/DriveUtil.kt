package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.max
import kotlin.math.min

/** A utility script made to use the drive system on the robot */
class DriveUtil (
    hardwareMap : HardwareMap,
    private val drivePower : Double = 0.2
) {

    private val lDrive : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lDrive")
    private val rDrive : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rDrive")

    var velocityModeInitialized = false
    var velocityPowerScale = 1.0

    // Order is kP kI kD kF
    val pidf = arrayOf(11.0, 0.0, 0.15, 0.035).toDoubleArray()

    init {
        rDrive.direction = DcMotorSimple.Direction.REVERSE
        ensureVelocityMode()
        setPIDF()
    }

    /** A function to set powers to the motors based off of a double for left and right power */
    fun setDrivePowers(leftPower: Double, rightPower: Double) {
        setMotorVelocityFromPseudoPower(lDrive, leftPower)
        setMotorVelocityFromPseudoPower(rDrive, rightPower)
    }

    fun setPIDF() {
        listOf(lDrive, rDrive)
            .forEach { it.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]) }
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
}