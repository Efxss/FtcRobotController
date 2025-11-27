package org.firstinspires.ftc.teamcode.OldCode

import android.util.Size
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Thread.sleep
import kotlin.math.max
import kotlin.math.min

@Disabled
class TeleOp : OpMode() {
    private var panels: TelemetryManager? = null
    private lateinit var frontLeft: DcMotorEx
    private lateinit var frontRight: DcMotorEx
    private lateinit var backLeft: DcMotorEx
    private lateinit var backRight: DcMotorEx
    private lateinit var outTake1: DcMotorEx
    private lateinit var outTake2: DcMotorEx
    private lateinit var intakeServo1: CRServo
    private lateinit var intakeServo2: CRServo
    private lateinit var bowlServo : Servo
    private lateinit var camServo  : Servo
    val lP1 = 0.059
    val lP2 = 0.13
    val lP3 = 0.204
    val fP1 = 0.167
    val fP2 = 0.02
    val fP3 = 0.0945
    var cLP = 1
    var driveSpeed = false
    var servoSpeed = 0.0
    var moveMode   = 0
    var held = 0
    val ord = arrayOf("N", "N", "N")
    val eord = arrayOf("", "", "")
    private var lastDpadLeftState = false
    private var lastDpadRightState = false
    private var visionPortal: VisionPortal? = null
    private var tagProcessor: AprilTagProcessor? = null
    private var anyTagSeen: Boolean = false
    private var currentTagIds: List<Int> = emptyList()
    data class Target(
        val tx: Double,    // normalized X (-1..1)
        val ty: Double,    // normalized Y (-1..1)
        val ta: Double,    // area (pixels)
        val colorId: Int,  // 1=green, 2=purple
        val width: Double,
        val height: Double
    )
    private lateinit var limelight: Limelight3A
    private val stride = 6
    @Volatile
    var velocityModeInitialized = false
    var velocityPowerScale = 0.85
    val p = 10.toDouble()
    val i = 3.toDouble()
    val d = 0.toDouble()
    val f = 8.toDouble()

    override fun init() {
        frontLeft        = hardwareMap.get(DcMotorEx::class.java, "frontLeft")
        frontRight       = hardwareMap.get(DcMotorEx::class.java, "frontRight")
        backLeft         = hardwareMap.get(DcMotorEx::class.java, "backLeft")
        backRight        = hardwareMap.get(DcMotorEx::class.java, "backRight")
        outTake1         = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2         = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        intakeServo1     = hardwareMap.get(CRServo::class.java, "intakeServo1")
        intakeServo2     = hardwareMap.get(CRServo::class.java, "intakeServo2")
        bowlServo        = hardwareMap.get(Servo::class.java, "BowlServo")
        camServo         = hardwareMap.get(Servo::class.java, "CamServo")
        listOf(frontRight, frontLeft, intakeServo2, outTake2)
            .forEach { it.direction = DcMotorSimple.Direction.REVERSE }
        listOf(backRight, backLeft)
            .forEach { it.direction = DcMotorSimple.Direction.FORWARD }
        listOf(frontRight, backRight, frontLeft, backLeft, outTake1, outTake2)
            .forEach { it.setVelocityPIDFCoefficients(p, i, d, f) }
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(100)     // fast polling
        limelight.pipelineSwitch(0)      // <- change to your pipeline slot
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults()
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(tagProcessor)
            .setCameraResolution(Size(1280, 720))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
        panels?.addLine("Limelight init complete.")
        panels = PanelsTelemetry.telemetry
        resetEncoders()
    }

    override fun start() {
        limelight.start()
        bowlServo.position = lP1
        camServo.position = 0.0
    }

    override fun loop() {
        val detections = tagProcessor?.detections.orEmpty()
        currentTagIds = detections.map { it.id }.toSet().toList().sorted()
        anyTagSeen = currentTagIds.isNotEmpty()

        for (id in currentTagIds) {
            when (id) {
                21 -> {
                    panels?.addLine("21")
                    eord[0] = "G"
                    eord[1] = "P"
                    eord[2] = "P"
                    break
                }
                22 -> {
                    panels?.addLine("22")
                    eord[0] = "P"
                    eord[1] = "G"
                    eord[2] = "P"
                    break
                }
                23 -> {
                    panels?.addLine("23")
                    eord[0] = "P"
                    eord[1] = "P"
                    eord[2] = "G"
                    break
                }
            }
        }

        var drive  = gamepad1.left_stick_y.toDouble() // Forward/backward
        var strafe = -gamepad1.right_stick_x.toDouble() // Left/right
        var rotate = (gamepad1.left_trigger - gamepad1.right_trigger).toDouble() // Rotation

        var frontLeftPower  = drive + strafe + rotate
        var frontRightPower = drive - strafe - rotate
        var backLeftPower   = drive - strafe + rotate
        var backRightPower  = drive + strafe - rotate

        if (gamepad1.left_bumper) {
            driveSpeed = false
        } else if (gamepad1.right_bumper) {
            driveSpeed = true
        }

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

        if (!driveSpeed) {
            frontLeft.velocity  = -frontLeftPower * 1000
            frontRight.velocity = frontRightPower * 1000
            backLeft.velocity   = backLeftPower * 1000
            backRight.velocity  = -backRightPower * 1000
        } else {
            frontLeft.velocity  = -frontLeftPower * 8000
            frontRight.velocity = frontRightPower * 8000
            backLeft.velocity   = backLeftPower * 8000
            backRight.velocity  = -backRightPower * 8000
        }

        /*if (gamepad1.dpad_left && !lastDpadLeftState) {
            moveMode -= 1
            if (moveMode < 0) {
                moveMode = 0
            }
        }
        lastDpadLeftState = gamepad1.dpad_left

        if (gamepad1.dpad_right && !lastDpadRightState) {
            moveMode += 1
            if (moveMode > 2) {
                moveMode = 2
            }
        }
        lastDpadRightState = gamepad1.dpad_right

        when (moveMode) {
            0 -> {
                bowlServo.position = lP1
            }
            1 -> {
                bowlServo.position = lP2
            }
            2 -> {
                bowlServo.position = lP3
            }
        }*/

        val result: LLResult? = limelight.latestResult
        if (result == null) {
            panels?.addLine("Limelight: no result yet")
            panels?.update()
            return
        }

        panels?.addData("Data age (ms)", result.staleness)

        // Read Python output FIRST — do not require result.isValid
        val py = result.pythonOutput
        if (py == null || py.isEmpty()) {
            // Fall back to built-in validity for info only
            panels?.addLine(
                if (result.isValid) "Python output empty (but built-in valid)"
                else "Python output empty"
            )
            panels?.update()
            return
        }

        // Parse 6-float tuples
        val stride = 6
        val targets = ArrayList<Target>(py.size / stride)
        var i = 0
        while (i + stride - 1 < py.size) {
            targets.add(
                Target(
                    tx = py[i + 0],
                    ty = py[i + 1],
                    ta = py[i + 2],
                    colorId = py[i + 3].toInt(),
                    width = py[i + 4],
                    height = py[i + 5]
                )
            )
            i += stride
        }

        val greenCount = targets.count { it.colorId == 1 }
        val purpleCount = targets.count { it.colorId == 2 }
        val best = targets.maxByOrNull { it.ta }

        panels?.addData("Targets", targets.size)
        panels?.addData("Green", greenCount)
        panels?.addData("Purple", purpleCount)

        best?.let {
            when (purpleCount) {
                1 -> {
                    if (it.width >= 250 && it.height >= 110 && it.ty > 0.55) {
                        // Fill the list array similar to color sensor logic
                        if (ord[0] == "N") {
                            // Check if already detected
                            if (ord[0] == "P" || ord[1] == "P" || ord[2] == "P") {
                                return
                            }
                            ord[0] = "P"

                            // Update servo position
                            if (cLP == 1) {
                                bowlServo.position = lP2
                                cLP = 2
                            }
                        } else if (ord[1] == "N") {
                            // Check if already detected
                            if (ord[1] == "P" || ord[2] == "P") {
                                return
                            }
                            ord[1] = "P"

                            // Update servo position
                            if (cLP == 2) {
                                bowlServo.position = lP3
                                cLP = 3
                            }
                        } else if (ord[2] == "N") {
                            // Check if already detected
                            if (ord[2] == "P") {
                                return
                            }
                            ord[2] = "P"

                            // No more servo positions (already at lP3)
                        } else {
                            // List is full
                            held = 1
                            return
                        }

                        sleep(800)
                    }
                }
            }
            when (greenCount) {
                1 -> {
                    if (it.width >= 250 && it.height >= 110 && it.ty > 0.55) {
                        // Fill the list array for green pieces
                        if (ord[0] == "N") {
                            // Check if green already detected
                            if (ord[0] == "G" || ord[1] == "G" || ord[2] == "G") {
                                return
                            }
                            ord[0] = "G"

                            // Update servo position
                            if (cLP == 1) {
                                bowlServo.position = lP2
                                cLP = 2
                            }
                        } else if (ord[1] == "N") {
                            // Check if green already detected
                            if (ord[0] == "G" || ord[1] == "G" || ord[2] == "G") {
                                return
                            }
                            ord[1] = "G"

                            // Update servo position
                            if (cLP == 2) {
                                bowlServo.position = lP3
                                cLP = 3
                            }
                        } else if (ord[2] == "N") {
                            // Check if green already detected
                            if (ord[0] == "G" || ord[1] == "G" || ord[2] == "G") {
                                return
                            }
                            ord[2] = "G"

                            // No more servo positions (already at lP3)
                        } else {
                            // List is full
                            held = 1
                            return
                        }

                        sleep(800)
                    }
                }
            }

            panels?.addData("order 1", ord[0])
            panels?.addData("order 2", ord[1])
            panels?.addData("order 3", ord[2])
            panels?.addData("expected order 1", eord[0])
            panels?.addData("expected order 2", eord[1])
            panels?.addData("expected order 3", eord[2])
            panels?.addLine("Best (by area)")
            panels?.addData(" colorId", it.colorId)
            panels?.addData(" tx(norm)", it.tx)
            panels?.addData(" ty(norm)", it.ty)
            panels?.addData(" area(px)", it.ta)
            panels?.addData(" W", it.width)
            panels?.addData(" H", it.height)
        }

        when (held) {
            0 -> { /* Do nothing */ }
            1 -> {
                setMotorVelocityFromPseudoPower(outTake1, 0.2)
                setMotorVelocityFromPseudoPower(outTake2, 0.2)
                sleep(1000)
                if (ord[0] != "N" && ord[1] != "N" && ord[2] != "N") {
                    if (eord[0] == "G" && eord[1] == "P" && eord[2] == "P") {
                        if (ord.contentEquals(eord)) {
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                        if (ord[0] == "P" && ord[1] == "P" && ord[2] == "G") {
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                        if (ord[0] == "P" && ord[1] == "G" && ord[2] == "P") {
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                    }
                    if (eord[0] == "P" && eord[1] == "G" && eord[2] == "P") {
                        if (ord.contentEquals(eord)) {
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                        if (ord[0] == "P" && ord[1] == "P" && ord[2] == "G") {
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                        if (ord[0] == "G" && ord[1] == "P" && ord[2] == "P") {
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                    }
                    if (eord[0] == "P" && eord[1] == "P" && eord[2] == "G") {
                        if (ord.contentEquals(eord)) {
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                        if (ord[0] == "P" && ord[1] == "G" && ord[2] == "P") {
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                        if (ord[0] == "G" && ord[1] == "P" && ord[2] == "P") {
                            sleep(5000)
                            bowlServo.position = fP2
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP3
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            bowlServo.position = fP1
                            sleep(1500)
                            camServo.position = 0.5
                            sleep(500)
                            camServo.position = 0.0
                            sleep(5000)
                            eord[0] = ""
                            eord[1] = ""
                            eord[2] = ""
                            ord[0] = "N"
                            ord[1] = "N"
                            ord[2] = "N"
                            cLP = 1
                            return
                        }
                    }
                }
                sleep(1000)
                setMotorVelocityFromPseudoPower(outTake1, 0.0)
                setMotorVelocityFromPseudoPower(outTake2, 0.0)
                bowlServo.position = lP1
                sleep(1000)
                held = 0
                return
            }
        }
        panels?.addData("MoveMode", moveMode)
        panels?.addData("BowlServo", bowlServo.position)
        panels?.update()
    }

    fun resetEncoders() {
        frontLeft.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontLeft.mode  = DcMotor.RunMode.RUN_USING_ENCODER
        frontRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backLeft.mode   = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeft.mode   = DcMotor.RunMode.RUN_USING_ENCODER
        backRight.mode  = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRight.mode  = DcMotor.RunMode.RUN_USING_ENCODER
        frontLeft.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeft.zeroPowerBehavior   = DcMotor.ZeroPowerBehavior.BRAKE
        backRight.zeroPowerBehavior  = DcMotor.ZeroPowerBehavior.BRAKE
    }

    private fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            outTake1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            outTake2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityModeInitialized = true
        }
    }

    private fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM // no-load
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }

    private fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        val tps = powerToTicksPerSecond(motor, power)
        motor.velocity = tps
    }

    override fun stop() {
        limelight.stop()
        panels?.addData("Status", "Stopped")
        panels?.update()
    }
}