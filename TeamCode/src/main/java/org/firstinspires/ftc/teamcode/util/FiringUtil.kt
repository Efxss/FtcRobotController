package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS

/** A utility script made for entering the firing sequence */
class FiringUtil(
    hardwareMap : HardwareMap,
    val spinDexer : SpinDexerSS,
    val cam : CamSS,
    val maxPower : Double,
    val velocityPowerScale : Double,
    val pidf : PIDFCoefficients
) {
    private val flyWheel: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "flyWheel")
    private val timer = ElapsedTime()

    init {
        flyWheel.direction = DcMotorSimple.Direction.FORWARD
        flyWheel.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private enum class FiringState {
        IDLE, SPIN_UP, MOVE_DEXER, WAIT_DEXER, FIRE_CAM, WAIT_CAM_FIRE, CAM_HOME, WAIT_CAM_HOME, DONE
    }

    private enum class FastFiringState {
        IDLE, SPIN_UP, MOVE_DEXER, WAIT_DEXER, FIRE_CAM, WAIT_CAM_FIRE, CAM_HOME, WAIT_CAM_HOME, DONE
    }
    private var state = FiringState.IDLE
    private var fastState = FastFiringState.IDLE
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
                MathUtil.setMotorVelocityFromPseudoPower(flyWheel, maxPower, velocityPowerScale, pidf)
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

    fun fastUpdate() {
        when (fastState) {
            FastFiringState.IDLE -> { }

            FastFiringState.SPIN_UP -> {
                MathUtil.setMotorVelocityFromPseudoPower(flyWheel, maxPower, velocityPowerScale, pidf)
                currentStep = 0
                fastState = FastFiringState.MOVE_DEXER
                timer.reset()
            }

            FastFiringState.MOVE_DEXER -> {
                if (timer.milliseconds() >= 650) {
                    steps[currentStep]()
                    timer.reset()
                    fastState = FastFiringState.WAIT_DEXER
                }
            }

            FastFiringState.WAIT_DEXER -> {
                if (timer.milliseconds() >= 1500) {
                    cam.fire()
                    timer.reset()
                    fastState = FastFiringState.FIRE_CAM
                }
            }

            FastFiringState.FIRE_CAM -> {
                if (timer.milliseconds() >= 350) {
                    cam.home()
                    timer.reset()
                    fastState = FastFiringState.WAIT_CAM_HOME
                }
            }

            FastFiringState.WAIT_CAM_HOME -> {
                if (timer.milliseconds() >= 150) {
                    currentStep++
                    if (currentStep < steps.size) {
                        timer.reset()
                        fastState = FastFiringState.MOVE_DEXER
                    } else {
                        fastState = FastFiringState.DONE
                    }
                }
            }

            FastFiringState.DONE -> {
                spinDexer.loadOne(true)
                flyWheel.power = 0.0
                fastState = FastFiringState.IDLE
            }

            else -> {}
        }
    }
    /** Call this function to enter the firing sequence */
    fun startFiring(button: Boolean) {
        if (button && state == FiringState.IDLE) {
            state = FiringState.SPIN_UP
        }
    }

    fun startFastFiring(button: Boolean) {
        if (button && fastState == FastFiringState.IDLE) {
            fastState = FastFiringState.SPIN_UP
        }
    }
    /** This function will return true if the robot is firing else it will return false */
    fun isFiring(): Boolean = state != FiringState.IDLE

    /** This function will return the power of the flywheel */
    fun flyWheelPower(): Double = flyWheel.power

    /** This function will return the velocity of the flywheel */
    fun flyWheelVelocity(): Double = flyWheel.velocity
}