package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit

/**
 * A utility for getting and sending information to the REV hub (Like bulk reads)
 *
 * Bulk reads consolidate all sensor reads on a hub into a single command,
 * cutting loop times significantly when multiple hardware reads happen per cycle.
 *
 * Uses MANUAL mode so exactly one bulk read occurs per loop cycle.
 * Call [clearCache] at the top of every loop() iteration.
 */
class HubUtil(hardwareMap : HardwareMap) {

    private val hubs : List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)

    init {
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
    }

    /** Clears the bulk read cache on every hub. Call this once at the start of each loop() cycle. */
    fun clearCache() = hubs.forEach { it.clearBulkCache() }

    /** Calling this function will return the temperature of the hub in Celsius
     * @see [getTempFahrenheit]
     * @see [getTempKelvin] */
    fun getTempCelsius() = hubs.forEach { it.getTemperature(TempUnit.CELSIUS) }

    /** Calling this function will return the temperature of the hub in Fahrenheit
     * @see [getTempCelsius]
     * @see [getTempKelvin] */
    fun getTempFahrenheit() = hubs.forEach { it.getTemperature(TempUnit.FARENHEIT) }

    /** Calling this function will return the temperature of the hub in Kelvins
     * @see [getTempCelsius]
     * @see [getTempFahrenheit] */
    fun getTempKelvin() = hubs.forEach { it.getTemperature(TempUnit.KELVIN) }
}
