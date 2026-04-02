package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.MathUtil

class SpinDexerSS (
    hardwareMap : HardwareMap
) {
    companion object {
        const val LOAD_ONE = 0.021
        const val LOAD_TWO = 0.092
        const val LOAD_THREE = 0.164
        const val FIRE_ONE = 0.128
        const val FIRE_TWO = 0.195
        const val FIRE_THREE = 0.058
    }

    private val spinDexer : Servo = hardwareMap.get(Servo::class.java, "SpinDexer")

    fun loadOne(button: Boolean) {
        if (button) spinDexer.position = LOAD_ONE
    }

    fun loadTwo(button: Boolean) {
        if (button) spinDexer.position = LOAD_TWO
    }

    fun loadThree(button: Boolean) {
        if (button) spinDexer.position = LOAD_THREE
    }
    fun fireOne() {
        spinDexer.position = FIRE_ONE
    }

    fun fireTwo() {
        spinDexer.position = FIRE_TWO
    }

    fun fireThree() {
        spinDexer.position = FIRE_THREE
    }

    fun position(): String {
        val spinDexerPos = spinDexer.position

        return when {
            MathUtil.closeTo(spinDexerPos, LOAD_ONE) -> "Load One"
            MathUtil.closeTo(spinDexerPos, LOAD_TWO) -> "Load Two"
            MathUtil.closeTo(spinDexerPos, LOAD_THREE) -> "Load Three"
            MathUtil.closeTo(spinDexerPos, FIRE_ONE) -> "Fire One"
            MathUtil.closeTo(spinDexerPos, FIRE_TWO) -> "Fire Two"
            MathUtil.closeTo(spinDexerPos, FIRE_THREE) -> "Fire Three"
            else -> "Unknown"
        }
    }

    fun rawPosition() : Double = spinDexer.position
}