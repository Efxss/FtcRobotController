package org.firstinspires.ftc.teamcode.config.util

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import kotlin.math.abs

/**
 * A utility for getting and sending information to the REV hub (Like bulk reads)
 *
 * Bulk reads consolidate all sensor reads on a hub into a single command,
 * cutting loop times significantly when multiple hardware reads happen per cycle.
 *
 * Uses MANUAL mode so exactly one bulk read occurs per loop cycle.
 * Call [clearCache] at the top of every loop() iteration.
 */
class HubUtil (
    hardwareMap: HardwareMap
) {

    private val hubs: List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)
    private var volts: Double = 0.0
    private var lastVolts: Double = 0.0
    private var voltCache: Double = 0.0

    init {
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
    }

    /** Clears the bulk read cache on every hub. Call this once at the start of each loop() cycle. */
    fun clearCache() = hubs.forEach { it.clearBulkCache() }
    fun setVoltCache(voltCache: Double) { this.voltCache = voltCache }
    fun getVolts(voltageUnit: VoltageUnit): Double {
        require(voltCache != 0.0)
        if (abs(this.volts - this.lastVolts) > this.voltCache || this.volts == 0.0) {
            this.lastVolts = this.volts
            hubs.forEach { this.volts = it.getInputVoltage(voltageUnit) }
        }
        return this.volts
    }
}
