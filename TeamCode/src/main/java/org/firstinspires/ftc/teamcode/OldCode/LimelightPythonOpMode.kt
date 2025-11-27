package org.firstinspires.ftc.teamcode.OldCode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import java.lang.Thread.sleep

@Disabled
@TeleOp(name = "Limelight 3A — Python Output (OpMode)", group = "Vision")
class LimelightPythonOpMode : OpMode() {
    private var panels: TelemetryManager? = null

    val lP1 = 0.059
    val lP2 = 0.13
    val lP3 = 0.204
    val fP1 = 0.167
    val fP2 = 0.02
    val fP3 = 0.0945
    var cLP = 1

    val ord = arrayOf("N", "N", "N")
    val eord = arrayOf("P", "G", "P")

    lateinit var servo : Servo

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

    override fun init() {
        panels = PanelsTelemetry.telemetry
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        servo = hardwareMap.get(Servo::class.java, "Servo")
        limelight.setPollRateHz(100)     // fast polling
        limelight.pipelineSwitch(0)      // <- change to your pipeline slot
        panels?.addLine("Limelight init complete.")
    }

    override fun start() {
        limelight.start()
        servo.position = lP1
    }

    override fun loop() {
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
                    if (it.width >= 280 && it.height >= 170) {
                        // Fill the list array similar to color sensor logic
                        if (ord[0] == "N") {
                            // Check if already detected
                            if (ord[0] == "P" || ord[1] == "P" || ord[2] == "P") {
                                return
                            }
                            ord[0] = "P"

                            // Update servo position
                            if (cLP == 1) {
                                servo.position = lP2
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
                                servo.position = lP3
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
                            return
                        }

                        sleep(800)
                    }
                }
            }
            when (greenCount) {
                1 -> {
                    if (it.width >= 280 && it.height >= 170) {
                        // Fill the list array for green pieces
                        if (ord[0] == "N") {
                            // Check if green already detected
                            if (ord[0] == "G" || ord[1] == "G" || ord[2] == "G") {
                                return
                            }
                            ord[0] = "G"

                            // Update servo position
                            if (cLP == 1) {
                                servo.position = lP2
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
                                servo.position = lP3
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
                            return
                        }

                        sleep(800)
                    }
                }
            }

            panels?.addData("Order 1", ord[0])
            panels?.addData("Order 2", ord[1])
            panels?.addData("Order 3", ord[2])
            panels?.addLine("Best (by area)")
            panels?.addData(" colorId", it.colorId)
            panels?.addData(" tx(norm)", it.tx)
            panels?.addData(" ty(norm)", it.ty)
            panels?.addData(" area(px)", it.ta)
            panels?.addData(" W", it.width)
            panels?.addData(" H", it.height)
        }

        /*if (ord.contentEquals(eord)) {
            sleep(5000)
            servo.position = fP1
            sleep(5000)
            servo.position = fP2
            sleep(5000)
            servo.position = fP3
            sleep(5000)
            stop()
        }*/

        if (ord[0] != "N" && ord[1] != "N" && ord[2] != "N") {
            if (eord[0] == "G" && eord[1] == "P" && eord[2] == "P") {
                if (ord.contentEquals(eord)) {
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    stop()
                }
                if (ord[0] == "P" && ord[1] == "P" && ord[2] == "G") {
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    stop()
                }
                if (ord[0] == "P" && ord[1] == "G" && ord[2] == "P") {
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    stop()
                }
            }
            if (eord[0] == "P" && eord[1] == "G" && eord[2] == "P") {
                if (ord.contentEquals(eord)) {
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    stop()
                }
                if (ord[0] == "P" && ord[1] == "P" && ord[2] == "G") {
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    stop()
                }
                if (ord[0] == "G" && ord[1] == "P" && ord[2] == "P") {
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    stop()
                }
            }
            if (eord[0] == "P" && eord[1] == "P" && eord[2] == "G") {
                if (ord.contentEquals(eord)) {
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    stop()
                }
                if (ord[0] == "P" && ord[1] == "G" && ord[2] == "P") {
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    stop()
                }
                if (ord[0] == "G" && ord[1] == "P" && ord[2] == "P") {
                    sleep(5000)
                    servo.position = fP2
                    sleep(5000)
                    servo.position = fP3
                    sleep(5000)
                    servo.position = fP1
                    sleep(5000)
                    stop()
                }
            }
            /*if (ord[0] == "G" && ord[1] == "P" && ord[2] == "P") {
                sleep(5000)
                servo.position = fP3
                sleep(5000)
                servo.position = fP2
                sleep(5000)
                servo.position = fP1
                sleep(5000)
                stop()
            } else if (ord[0] == "P" && ord[1] == "G" && ord[2] == "P") {
                sleep(5000)
                servo.position = fP1
                sleep(5000)
                servo.position = fP3
                sleep(5000)
                servo.position = fP2
                sleep(5000)
                stop()
            }*/
        }

        panels?.update()
    }
    override fun stop() {
        limelight.stop()
        panels?.addData("Status", "Stopped")
        panels?.update()
    }
}