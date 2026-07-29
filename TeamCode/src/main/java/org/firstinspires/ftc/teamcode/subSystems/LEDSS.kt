package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

class LEDSS(
    hardwareMap: HardwareMap
) {
    private var statusLEDR: DigitalChannel = hardwareMap.get(DigitalChannel::class.java, "LEDR")
    private var statusLEDG: DigitalChannel = hardwareMap.get(DigitalChannel::class.java, "LEDG")
    fun ledOn() {listOf(statusLEDR, statusLEDG).forEach{it.state = true}}
    fun ledOff(){listOf(statusLEDR, statusLEDG).forEach {it.state = false}}
}