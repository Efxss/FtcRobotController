package org.firstinspires.ftc.teamcode.OldCode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import java.lang.Thread.sleep

@Disabled
class STEMTeleOP : OpMode() {
    val lP1 = 0.059
    val lP2 = 0.13
    val lP3 = 0.204
    val fP1 = 0.167
    val fP2 = 0.02
    val fP3 = 0.0945
    var servoSpeed = 0.0
    var held = 0
    private lateinit var intakeServo1: CRServo
    private lateinit var intakeServo2: CRServo
    private lateinit var outTake1: DcMotorEx
    private lateinit var outTake2: DcMotorEx
    private lateinit var bowlServo: Servo
    private lateinit var outServo: Servo
    override fun init() {
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        intakeServo2 = hardwareMap.get(CRServo::class.java, "intakeServo2")
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        bowlServo  = hardwareMap.get(Servo::class.java, "bowlServo")
        outServo  = hardwareMap.get(Servo::class.java, "outServo")
        intakeServo2.direction = DcMotorSimple.Direction.REVERSE
        outTake2.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun start() {
        bowlServo.position = lP1
        outServo.position = 0.0
    }

    override fun loop() {
        if (gamepad1.cross) {
            servoSpeed = 1.0
            intakeServo1.power = servoSpeed
            intakeServo2.power = servoSpeed
        } else if (gamepad1.circle) {
            servoSpeed = -1.0
            intakeServo1.power = servoSpeed
            intakeServo2.power = servoSpeed
        } else {
            servoSpeed = 0.0
            intakeServo1.power = servoSpeed
            intakeServo2.power = servoSpeed
        }
        if (gamepad1.triangle) {
            held = 1
        }
        when (held) {
            0 -> { /* Do nothing */ }
            1 -> {
                outTake1.power = 0.3
                outTake2.power = 0.3
                sleep(1000)
                bowlServo.position = fP1
                sleep(1000)
                outServo.position = 0.25
                sleep(500)
                outServo.position = 0.0
                sleep(1000)
                outTake1.power = 0.0
                outTake2.power = 0.0
                bowlServo.position = lP1
                sleep(1000)
                held = 0
                return
            }
        }
    }
}