@file:Suppress("unused") // Stop Android Studio from complaining that this helper script is not currently being used

package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.max

private lateinit var FL     : DcMotorEx
private lateinit var FR     : DcMotorEx
private lateinit var BL     : DcMotorEx
private lateinit var BR     : DcMotorEx
private lateinit var OUT1   : DcMotorEx
private lateinit var OUT2   : DcMotorEx
private var tagProcessor    : AprilTagProcessor? = null
private const val CamW      = 1280      // Webcam width
private const val CamH      = 720       // Webcam height
private const val CamFOV    = 76        // Webcam horizontal FOV in degrees     (May not need, BUT KEEP FOR NOW!)
private const val RealSize  = 8.125     // Real size of the AprilTag in inches  (May not need, BUT KEEP FOR NOW!)
private const val basePower = 1.20      // The base power TODO: TUNE THIS!!!
private const val PowerMult = -0.001778 // The linear power difference TODO: TUNE THIS!!!

enum class Tags(val value: Int) {
    BLUE_GOAL(21),
    RED_GOAL(24)
}

/**
 * **A simple helper script to simplify Depot locking & firing**
 */
object CenterTag {

    /**
     * **Passes the drive motors, outtake motors, and webcam to CenterTag**
     *
     * @param fl The front left motor (Should just be `fl = frontLeft`)
     * @param bl The back left motor (Should just be `bl = backLeft`)
     * @param fr The front right motor (Should just be `fr = frontRight`)
     * @param br The back right motor (Should just be `br = backRight`)
     * @param out1 The first outtake motor (If outtake is spinning backwards, swap with [out2])
     * @param out2 The second outtake motor (If outtake is spinning backwards, swap with []out1])
     * @param webcam The *goBILDA* webcam
     */
    fun setup(fl: DcMotorEx, fr: DcMotorEx, bl: DcMotorEx, br: DcMotorEx, out1: DcMotorEx, out2: DcMotorEx, webcam: AprilTagProcessor) {
        FL = fl
        FR = fr
        BL = bl
        BR = br
        OUT1 = out1
        OUT2 = out2
        tagProcessor = webcam
    }

    /**
     * **Locks onto the selected AprilTag and, optionally, calculates & sets the outtake power**
     *
     * @param tag The selected Depot AprilTag to lock on to
     * @param reps How many times the robot should retry searching for the AprilTag if it's not immediately found
     * @param fire Weather or not the robot should spin up the outtake to the calculated power
     *
     * @return Weather or not the robot could detect the AprilTag in the set [reps]
     */
    fun lock(tag: Tags, reps: Int, fire: Boolean): Boolean {
        var tagDetected = false
        val centerX = (CamW.toDouble()) / 2
        var target: AprilTagDetection? = null

        // Try to search for the tag [reps] times before giving up
        repeat(reps) {
            for (det in tagProcessor!!.detections) {
                if (det.id == tag.value) {
                    target = det
                    tagDetected = true
                    break
                }
            }
        }

        if (target == null) {
            tagDetected = false
        }

        if (tagDetected) {
            var targetError = target!!.center.x - centerX
            while (abs(targetError) > 2) {
                // Update the target
                target = null
                for (det in tagProcessor!!.detections) {
                    if (det.id == tag.value) {
                        target = det
                        break
                    }
                }
                if (target == null) {
                    // Use last known [targetError] ([targetError] does not change)
                } else {
                    targetError = target.center.x - centerX
                }

                // Calculate the power with (hopefully) smooth tracking
                var power = max(abs(targetError) / 1500, 0.05) // TODO: CAN BE CHANGED TO VAL???

                // Rotate right
                if (targetError > 3) {
                    FL.power = -power
                    FR.power = -power
                    BL.power = -power
                    BR.power = -power
                    // Rotate left
                } else if (targetError < -3) {
                    FL.power = power
                    FR.power = power
                    BL.power = power
                    BR.power = power
                }

                Thread.sleep(25) // TODO: MAY NOT NEED THIS? (KEPT FROM OLD SCRIPT)
            }
            FL.power = 0.0
            FR.power = 0.0
            BL.power = 0.0
            BR.power = 0.0
        }

        if (tagDetected && target != null && fire) {
            // TODO: IF [pose.z] IS OKAY, USE THIS:
            val outtakePower = (PowerMult * target!!.ftcPose.z)
            // TODO: IF [pose.z] SUCKS, USE THIS:
            /* BTW:
                corners[0] = top-left (x, y)
                corners[1] = top-right (x, y)
                corners[2] = bottom-right (x, y)
                corners[3] = bottom-left (x, y)
             */
            // Get the corner Y positions
            val tlY: Double = target.corners[0].y
            val trY: Double = target.corners[1].y
            val brY: Double = target.corners[2].y
            val blY: Double = target.corners[3].y

            // Calculate the height on both sides, then average them
            val tagSize = ((blY - tlY) + (brY - trY)) / 2

            // Calculate & set the outtake power
            val outtakePower = (PowerMult * tagSize) + basePower // TODO: MAY NEED TO BE QUADRATIC? <-- TEST
            // TODO: MAY NEED TO INVERT!!!
            OUT1.velocity = outtakePower  // TODO: ADD POWER TO VELOCITY CONVERSION!!
            OUT2.velocity = -outtakePower // TODO: ADD POWER TO VELOCITY CONVERSION!!
        }

        return tagDetected
    }
}
