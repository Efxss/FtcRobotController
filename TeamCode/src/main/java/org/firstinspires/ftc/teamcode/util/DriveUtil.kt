package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class DriveUtil(
    hardwareMap: HardwareMap,
    private val drivePower: Double = 0.2,
    private val deadzone: Float = 0.2f
) {

    private val lDrive: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "lDrive")
    private val rDrive: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rDrive")

    init {
        rDrive.direction = DcMotorSimple.Direction.REVERSE
    }

    fun tankDrive(leftStick: Float, rightStick: Float) {
        lDrive.power = processInput(leftStick)
        rDrive.power = processInput(rightStick)
    }

    private fun processInput(input: Float): Double {
        return when {
            input > deadzone -> drivePower
            input < -deadzone -> -drivePower
            else -> 0.0
        }
    }
}