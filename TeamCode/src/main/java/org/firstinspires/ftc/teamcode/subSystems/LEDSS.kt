package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

class LEDSS(hardwareMap: HardwareMap) {
    private var statusLEDR:DigitalChannel=hardwareMap.get(DigitalChannel::class.java, "LEDR")
    private var statusLEDG:DigitalChannel=hardwareMap.get(DigitalChannel::class.java, "LEDG")
    init {listOf(statusLEDR,statusLEDG).forEach{it.mode=DigitalChannel.Mode.OUTPUT}
        listOf(statusLEDR,statusLEDG).forEach{it.state=true}}
    fun ledOn(){listOf(statusLEDR,statusLEDG).forEach{it.state=false}}
    fun ledOff(){listOf(statusLEDR,statusLEDG).forEach{it.state=true}}
}