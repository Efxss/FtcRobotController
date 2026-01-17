package org.firstinspires.ftc.teamcode.util

object Config {

    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.021
        const val LOAD_P2 = 0.087
        const val LOAD_P3 = 0.158

        // Firing/dispensing positions
        const val FIRE_P1 = 0.128
        const val FIRE_P2 = 0.195
        const val FIRE_P3 = 0.058

        // Camera servo positions
        const val CAM_OPEN = 0.44
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }

    object EndGame {
        const val LIFTMAX = 11400
        const val SLOWMODE = 1000
        const val NORMALSPEED = 1.0
        const val SLOWSPEED = 0.2
    }

    object DriveCaps {
        const val NORMAL_MAX = 0.80
        const val SLOW_MAX = 0.30
        const val DISPENSE_MAX = 0.15
    }
}

