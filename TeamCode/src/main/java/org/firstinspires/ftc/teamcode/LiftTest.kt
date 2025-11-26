package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.random.Random

// Here are the values for lift pos:
/*
SAFE MAX:    11400 <-- NORMAL MAX
TENSION MAX: 11900 <-- ONLY FOR TUNING!
 */


@Configurable
object RobotConstants {
    @JvmField var targetPos:    Int = 20
    @JvmField var threshold:    Int = 1

    @JvmField var power:        Double = 0.8
    @JvmField var topLimit:     Int = 11400
    @JvmField var bottomLimit:  Int = 0

    @JvmField var randMove:     Boolean = false
}

@Suppress("unused")
@TeleOp(name = "LiftTest", group = "TEST")
class LiftTest : OpMode() {

    private enum class STATUS {IDLE, WAKING, MOVING_TO_POS, STOPPING}

    // MUST INCLUDE (2x)
    private lateinit var liftLeft: DcMotorEx
    private lateinit var liftRight: DcMotorEx

    @Suppress("SpellCheckingInspection")
    private var timeset: Long = 0
    @Suppress("SpellCheckingInspection")
    private var randset: Boolean = false
    private var panels: TelemetryManager? = null

  // MUST INCLUDE!
    override fun init() {
        panels = PanelsTelemetry.telemetry
        panels?.addLine("⚠\uFE0F WARNING: THIS IS A TEST, ONLY SET CONSTANTS AFTER THOROUGH EVALUATION. FAILURE TO DO THIS WILL RESULT IN PHYSICAL ROBOT DAMAGE! ⚠\uFE0F")
        panels?.update()
        // MUST INCLUDE FROM HERE...
        liftLeft = hardwareMap.get(DcMotorEx::class.java, "liftLeft")
        liftRight = hardwareMap.get(DcMotorEx::class.java, "liftRight")

        liftLeft.direction = DcMotorSimple.Direction.REVERSE

        liftLeft.targetPosition = 0
        liftRight.targetPosition = 0

        liftLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftRight.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        // ...TO HERE IN SETUP
    }

    override fun loop() {

        checkVars()
        if (RobotConstants.randMove) {
            if (abs(timeset - System.currentTimeMillis()) >= 5000) {
                RobotConstants.targetPos = Random.nextInt(RobotConstants.bottomLimit, RobotConstants.topLimit + 1)
                randset = true
            }
        } else randset = false
        checkVars()

        if (!withinThreshold()) runToPos(RobotConstants.targetPos, liftLeft, liftRight)
        if (randset) {
            timeset = System.currentTimeMillis()
            randset = false
        }

        printData(STATUS.IDLE)
    }

    // MUST INCLUDE!
    private fun runToPos(motorTicks: Int, motor1: DcMotorEx, motor2: DcMotorEx) {
        printData(STATUS.WAKING)
        checkVars()
        // MUST INCLUDE FROM HERE...
        motor1.targetPosition = motorTicks
        motor2.targetPosition = motorTicks
        motor1.power = RobotConstants.power
        motor2.power = RobotConstants.power
        while (motor1.isBusy && motor2.isBusy) {
            motor1.power = RobotConstants.power
            motor2.power = RobotConstants.power

            checkVars()
            printData(STATUS.MOVING_TO_POS)
        }
        motor1.power = 0.0
        motor2.power = 0.0
        // ...TO HERE FOR SETTING THE LIFT POS
        printData(STATUS.STOPPING)
    }

    private fun withinThreshold(): Boolean {
        val left  = abs(liftLeft.currentPosition - RobotConstants.targetPos)  <= RobotConstants.threshold
        val right = abs(liftRight.currentPosition - RobotConstants.targetPos) <= RobotConstants.threshold

        return left && right
    }

    private fun printData(status: STATUS) {
        when (status) {
            STATUS.IDLE             -> panels?.addLine("STATUS:         \uD83D\uDE34 IDLE")
            STATUS.WAKING           -> panels?.addLine("STATUS:         \uD83D\uDE2E WAKING")
            STATUS.MOVING_TO_POS    -> panels?.addLine("STATUS:         \uD83C\uDFC3\u200D♂\uFE0F\u200D➡\uFE0F MOVING TO POS")
            STATUS.STOPPING         -> panels?.addLine("STATUS:         \uD83E\uDD71 STOPPING")
        }
        panels?.addLine("Lift Left Pos:  ${liftLeft.currentPosition}")
        panels?.addLine("Lift Right Pos: ${liftRight.currentPosition}")
        panels?.addLine("Target Pos:     ${RobotConstants.targetPos}")
        panels?.addLine("Threshold:      ${RobotConstants.threshold}")
        panels?.addLine("")
        panels?.addLine("Power:          ${RobotConstants.power}")
        panels?.addLine("Top Limit:      ${RobotConstants.topLimit}")
        panels?.addLine("Bottom Limit:   ${RobotConstants.bottomLimit}")
        panels?.update()
    }

    private fun checkVars() {
        if (RobotConstants.targetPos.coerceIn(RobotConstants.bottomLimit, RobotConstants.topLimit) != RobotConstants.targetPos) {
            RobotConstants.targetPos = RobotConstants.targetPos.coerceIn(RobotConstants.bottomLimit, RobotConstants.topLimit)
        }
        if (RobotConstants.threshold < 0) {
            RobotConstants.threshold = 0
        }
        if (RobotConstants.power.coerceIn(0.0, 1.0) != RobotConstants.power) {
            RobotConstants.power = RobotConstants.power.coerceIn(0.0, 1.0)
        }
    }
}
