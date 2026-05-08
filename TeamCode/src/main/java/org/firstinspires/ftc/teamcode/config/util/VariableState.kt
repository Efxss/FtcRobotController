package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.geometry.Pose

/**
 * Holds state that needs to persist between OpModes (Auto -> TeleOp).
 * Because this is a Kotlin `object`, it lives in the JVM for the entire
 * Robot Controller process it survives the OpMode lifecycle but resets
 * on a Robot Controller restart.
 */
object VariableState {
    // Null means "no auto ran" -> TeleOp should fall back to a default pose.
    var endOfAutoPose: Pose? = null

    // Add anything else you want to hand off later, e.g.:
    // var alliance: Alliance = Alliance.RED
    // var motif: Motif? = null

    /** Call from TeleOp init if you want a clean slate after consuming the data. */
    fun clear() {
        endOfAutoPose = null
    }
}