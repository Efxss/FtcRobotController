package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients

/** A utility script made to use the drive system on the robot */
class DriveUtil (
    hardwareMap : HardwareMap,
    private val drivePower : Double = 0.2,
    val velocityPowerScale : Double,
    val pidf : PIDFCoefficients
) {

    private val lDrive : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lDrive")
    private val rDrive : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rDrive")

    init {
        rDrive.direction = DcMotorSimple.Direction.REVERSE
        lDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    /** A function to set powers to the motors based off of a double for left and right power */
    fun setDrivePowers(leftPower: Double, rightPower: Double) {
        MathUtil.setMotorVelocityFromPseudoPower(lDrive, leftPower, velocityPowerScale, pidf)
        MathUtil.setMotorVelocityFromPseudoPower(rDrive, rightPower, velocityPowerScale, pidf)
    }
}