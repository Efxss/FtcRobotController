package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS

class FiringUtil(
    hardwareMap: HardwareMap,
    private val spinDexer: SpinDexerSS,
    private val cam: CamSS
) {
    private val flyWheel: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "flyWheel")
    private val timer = ElapsedTime()

    private enum class FiringState {
        IDLE, SPIN_UP, MOVE_DEXER, WAIT_DEXER, FIRE_CAM, WAIT_CAM_FIRE, CAM_HOME, WAIT_CAM_HOME, DONE
    }

    private var state = FiringState.IDLE
    private var currentStep = 0
    private val steps: Array<() -> Unit> = arrayOf(
        { spinDexer.fireTwo() },
        { spinDexer.fireThree() },
        { spinDexer.fireOne() }
    )

    /** Call this every loop() iteration */
    fun update() {
        when (state) {
            FiringState.IDLE -> { }

            FiringState.SPIN_UP -> {
                flyWheel.power = 1.0
                currentStep = 0
                state = FiringState.MOVE_DEXER
                timer.reset()
            }

            FiringState.MOVE_DEXER -> {
                if (timer.milliseconds() >= 1500) {
                    steps[currentStep]()
                    timer.reset()
                    state = FiringState.WAIT_DEXER
                }
            }

            FiringState.WAIT_DEXER -> {
                if (timer.milliseconds() >= 1500) {
                    cam.fire()
                    timer.reset()
                    state = FiringState.FIRE_CAM
                }
            }

            FiringState.FIRE_CAM -> {
                if (timer.milliseconds() >= 1000) {
                    cam.home()
                    timer.reset()
                    state = FiringState.WAIT_CAM_HOME
                }
            }

            FiringState.WAIT_CAM_HOME -> {
                if (timer.milliseconds() >= 2000) {
                    currentStep++
                    if (currentStep < steps.size) {
                        timer.reset()
                        state = FiringState.MOVE_DEXER
                    } else {
                        state = FiringState.DONE
                    }
                }
            }

            FiringState.DONE -> {
                spinDexer.loadOne(true)
                flyWheel.power = 0.0
                state = FiringState.IDLE
            }

            else -> {}
        }
    }

    fun startFiring(button: Boolean) {
        if (button && state == FiringState.IDLE) {
            state = FiringState.SPIN_UP
        }
    }

    fun isFiring(): Boolean = state != FiringState.IDLE
    fun flyWheelPower(): Double = flyWheel.power
}