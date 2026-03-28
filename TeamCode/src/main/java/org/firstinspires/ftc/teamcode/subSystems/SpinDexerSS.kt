package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class SpinDexerSS(
    hardwareMap: HardwareMap
) {
    private val spinDexer: Servo = hardwareMap.get(Servo::class.java, "SpinDexer")

    fun loadOne(button: Boolean) {
        if (button) spinDexer.position = 0.021
    }

    fun loadTwo(button: Boolean) {
        if (button) spinDexer.position = 0.087
    }

    fun loadThree(button: Boolean) {
        if (button) spinDexer.position = 0.158
    }
    fun fireOne() {
        spinDexer.position = 0.128
    }

    fun fireTwo() {
        spinDexer.position = 0.195
    }

    fun fireThree() {
        spinDexer.position = 0.058
    }
}