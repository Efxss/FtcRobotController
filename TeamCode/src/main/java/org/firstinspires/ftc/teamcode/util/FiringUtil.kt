package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS
import java.lang.Thread.sleep

class FiringUtil(
    hardwareMap : HardwareMap
) {
    private val flyWheel : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "flyWheel")
    private var spinDexer : SpinDexerSS = SpinDexerSS(hardwareMap)
    private var cam : CamSS = CamSS(hardwareMap)
    private var isFiringVar : Boolean = false

    fun executeFiring(button: Boolean) {
        if (!button) return

        flyWheel.power = 1.0

        fireSequence(
            { spinDexer.fireTwo() },
            { spinDexer.fireThree() },
            { spinDexer.fireOne() }
        )

        spinDexer.loadOne(true)
        flyWheel.power = 0.0
        isFiringVar = false
    }

    fun fireSequence(vararg steps: () -> Unit) {
        isFiringVar = true
        for (step in steps) {
            sleep(1500)
            step()
            sleep(1500)
            cam.fireCam()
            sleep(2000)
        }
    }

    fun flyWheelPower() : Double {
        return flyWheel.power
    }

    fun isFiring() : Boolean {
        return isFiringVar
    }
}