package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import java.lang.Thread.sleep

@TeleOp
class RunLiftDown : OpMode() {
    private lateinit var liftLeft:  DcMotorEx
    private lateinit var liftRight: DcMotorEx
    val position = -1000
    override fun init() {
        liftLeft = hardwareMap.get(DcMotorEx::class.java, "liftLeft")
        liftRight = hardwareMap.get(DcMotorEx::class.java, "liftRight")
        listOf(liftLeft)
            .forEach { it.direction = DcMotorSimple.Direction.REVERSE }
        listOf(liftLeft, liftRight).forEach { motor ->
            motor.targetPosition = 125
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    override fun loop() {
        up()
        sleep(2000)
        down()
        sleep(2000)
        stop()
        sleep(500)
        terminateOpModeNow()
    }

    override fun stop() {
        listOf(liftLeft, liftRight).forEach { motor ->
            motor.power = 0.0
        }
    }

    fun up() {
        if (liftLeft.currentPosition < 125 || liftRight.currentPosition < 125) {
            listOf(liftLeft, liftRight).forEach { motor ->
                motor.power = 0.2
            }
        } else if (liftLeft.currentPosition > 125 || liftRight.currentPosition > 125) {
            listOf(liftLeft, liftRight).forEach { motor ->
                motor.power = 0.0
            }
        }
    }
    fun down() {
        listOf(liftLeft, liftRight).forEach { motor ->
            motor.targetPosition = position
        }
        if (liftLeft.currentPosition > position || liftRight.currentPosition > position) {
            listOf(liftLeft, liftRight).forEach { motor ->
                motor.power = -0.6
            }
        } else if (liftLeft.currentPosition < position || liftRight.currentPosition < position) {
            listOf(liftLeft, liftRight).forEach { motor ->
                motor.power = 0.0
            }
        }
    }
}