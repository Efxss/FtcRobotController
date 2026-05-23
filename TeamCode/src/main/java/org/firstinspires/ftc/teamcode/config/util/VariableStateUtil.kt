package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.geometry.Pose

/**
 * Holds state that needs to persist between OpModes (Auto -> TeleOp).
 * Because this is a Kotlin `object`, it lives in the JVM for the entire
 * Robot Controller process it survives the OpMode lifecycle but resets
 * on a Robot Controller restart.
 */
object VariableStateUtil {
    // Null means "no auto ran" -> TeleOp should fall back to a default pose.
    var endOfAutoPose: Pose? = null

    // Alliance var to pass from auto to TeleOP
    var alliance: Alliance = Alliance.BLUE

    /** Call from TeleOp init if you want a clean slate after consuming the data. */
    fun clearEndOfAutoPose() {
        endOfAutoPose = null
    }
}