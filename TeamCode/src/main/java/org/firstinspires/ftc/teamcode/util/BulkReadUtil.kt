package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * A utility for enabling bulk reads on all REV hubs.
 *
 * Bulk reads consolidate all sensor reads on a hub into a single command,
 * cutting loop times significantly when multiple hardware reads happen per cycle.
 *
 * Uses MANUAL mode so exactly one bulk read occurs per loop cycle.
 * Call [clearCache] at the top of every loop() iteration.
 */
class BulkReadUtil(hardwareMap : HardwareMap) {

    private val hubs : List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)

    init {
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
    }

    /** Clears the bulk read cache on every hub. Call this once at the start of each loop() cycle. */
    fun clearCache() = hubs.forEach { it.clearBulkCache() }
}
