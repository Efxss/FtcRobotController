package org.firstinspires.ftc.teamcode.helper

/**
 * A helper class that holds different objects for things to make scripts smaller
 */
class ObjectHelper {
    /**
     * A helper object that has different servo positons to call
     */
    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.004
        const val LOAD_P2 = 0.080
        const val LOAD_P3 = 0.153

        // Firing/dispensing positions
        const val FIRE_P1 = 0.118
        const val FIRE_P2 = 0.1885
        const val FIRE_P3 = 0.264

        // Camera servo positions
        const val CAM_OPEN = 0.5
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }
    /**
     * A helper object that holds all of the needed april tag IDS
     */
    object AprilTagIds {
        const val BLUE_DEPO = 20
        const val GPP_ORDER = 21
        const val PGP_ORDER = 22
        const val PPG_ORDER = 23
        const val RED_DEPO  = 24
    }
    /**
     * A helper object for vars to center on the depo
     */
    object DepoCenter {
        const val DESIRED_TAG_WIDTH_PX = 110
        const val ROTATE_POWER = 0.2
        const val CAM_WIDTH_PX = 1280
        const val CAM_HEIGHT_PX = 960
        const val CENTER_DEADZONE = 15
        const val KP_ROTATE = 0.003
        var OUTTAKE_SPEED = 0.20
    }
    object EndGame {
        const val LIFTMAX = 11400
        const val SLOWMODE = 1000
        const val NORMALSPEED = 0.6
        const val SLOWSPEED = 0.2
        var ISENDGAME = 0
    }
    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 1000L
        const val CAM_OPEN_DELAY = 400L
        const val CAM_CLOSE_DELAY = 1500L
        const val DETECTION_COOLDOWN = 1500L
        const val OUTTAKE_DELAY = 800L
    }
    object Color {

    }
}