package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.MathUtil

/** A SubSystem made for controlling the Spindexer on the robot */
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

    /** Calling this function rotate the Spindexer into load position one */
    fun loadOne(button: Boolean) {
        if (button) spinDexer.position = LOAD_ONE
    }

    /** Calling this function rotate the Spindexer into load position two */
    fun loadTwo(button: Boolean) {
        if (button) spinDexer.position = LOAD_TWO
    }

    /** Calling this function rotate the Spindexer into load position three */
    fun loadThree(button: Boolean) {
        if (button) spinDexer.position = LOAD_THREE
    }

    /** Calling this function rotate the Spindexer into fire position one */
    fun fireOne() {
        spinDexer.position = FIRE_ONE
    }

    /** Calling this function rotate the Spindexer into fire position two */
    fun fireTwo() {
        spinDexer.position = FIRE_TWO
    }

    /** Calling this function rotate the Spindexer into fire position three */
    fun fireThree() {
        spinDexer.position = FIRE_THREE
    }

    /** This function will return the position as a String if the position is within error of a known position */
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

    /** This function will return the raw position of the Spindexer as a double */
    fun rawPosition() : Double = spinDexer.position
}