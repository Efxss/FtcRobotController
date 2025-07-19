package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utils.sleep;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp
public class MentorChallengeOne extends ThreadOpMode {

    //region GLOBAL VARIABLES

    // WebCam Globals:
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    public static final double P = 10, I = 3, D = 0, F = 8;

    private Robot robot;

    // Trig [Triangulation] Globals:
    public static final double basketRange = 5; // Inches
    public static final double hCorrection = 0; // Degrees
    double X1, Y1, H1;
    double X2, Y2, H2;
    double TX, TY;

    public static final double CamRelOffsetX = -2.5;
    public static final double CamRelOffsetY = 2.125;

    public static final int CamW = 1280 / 2;
    public static final int CamH = 720 / 2;

    //endregion

    //region MainInit

    @Override
    public void mainInit() {

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                // specify your streaming resolution here
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Changed these to locals instead of class variables. -NP
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        SparkFunOTOS sparkFun = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontRight.setVelocityPIDFCoefficients(P, I, D, F);
        backRight.setVelocityPIDFCoefficients(P, I, D, F);
        frontLeft.setVelocityPIDFCoefficients(P, I, D, F);
        backLeft.setVelocityPIDFCoefficients(P, I, D, F);

        robot = Robot
                .getInstance(frontLeft, frontRight, backLeft, backRight)
                .setImu(sparkFun);

        configureOtos();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    //endregion

    //region Main

    @Override
    public void mainLoop() {
        telemetry.clearAll();

        TriangulateBasketPos();

        sleep(Long.MAX_VALUE);

        requestOpModeStop();

        HomePos();
        Collect("RED");
        ScoreBlock();

        HomePos();
        Collect("GREEN");
        ScoreBlock();

        HomePos();
        Collect("BLUE");
        ScoreBlock();

        HomePos();
        Collect("YELLOW");
        ScoreBlock();

        HomePos();

        requestOpModeStop();
    }

//endregion

    //region Exe Functions
    public void TriangulateBasketPos() {
        robot.goTo(0.2, "18", "18", "0");

        SparkFunOTOS.Pose2D pos = robot.getImu().getPosition();

        CenterTag(17);
        pos = robot.getImu().getPosition();

        // Get heading, inverted so CW is positive & CCW is negative
        H1 = -pos.h;
        // Get X & Y, accounting for camera offset
        double theta1 = Math.toRadians(90 - (H1 + hCorrection));
        X1 = pos.x + CamRelOffsetX * Math.cos(theta1) - CamRelOffsetY * Math.sin(theta1);
        Y1 = pos.y + CamRelOffsetX * Math.sin(theta1) + CamRelOffsetY * Math.cos(theta1);

        robot.strafeRelDist(0.2, 18);

        CenterTag(17);
        pos = robot.getImu().getPosition();

        // Get heading, inverted so CW is positive & CCW is negative
        H2 = -pos.h;
        // Get X & Y, accounting for camera offset
        double theta2 = Math.toRadians(90 - (H2 + hCorrection));
        X2 = pos.x + CamRelOffsetX * Math.cos(theta2) - CamRelOffsetY * Math.sin(theta2);
        Y2 = pos.y + CamRelOffsetX * Math.sin(theta2) + CamRelOffsetY * Math.cos(theta2);

        telemetry.addData("X1:", X1);
        telemetry.addData("Y1:", Y1);
        telemetry.addData("H1 (DEG):", "%.10f", H1);
        telemetry.addLine();

        telemetry.addData("X2:", X2);
        telemetry.addData("Y2:", Y2);
        telemetry.addData("H2 (DEG):", "%.10f", H2);
        telemetry.addLine();

        telemetry.addLine("Mathing...");
        telemetry.addLine(" !Be Aware: Could divide by 0 or <1e-6, very unlikely though!"); // POTENTIAL CATASTROPHIC ERROR: DIVIDE BY 0 OR <1e-6!!! IDK: How to fix/If even issue
        telemetry.update();

        /* OLD, POTENTIALLY INVALID, MATH
        double A = 180 - (H1 + H2);
        double a = Math.sqrt(Math.pow((X2 - X1), 2) + Math.pow((Y2 - Y1), 2)); we might need this line

        TY = ((a) / Math.tan(A)) * Math.cos(A);
        TX = TY * Math.tan(A);
         */

        // NEW, POTENTIALLY VALID, MATH:

        // Convert: Heading (DEG) --> Dir Vec (RAD)
        double dx1 = Math.cos(Math.toRadians(90 - (H1 + hCorrection)));
        double dy1 = Math.sin(Math.toRadians(90 - (H1 + hCorrection)));
        double dx2 = Math.cos(Math.toRadians(90 - (H2 + hCorrection)));
        double dy2 = Math.sin(Math.toRadians(90 - (H2 + hCorrection)));
        // Find the vector from point A (X1, Y1) to point B (X2, Y2)
        double DX = X2 - X1;
        double DY = Y2 - Y1;

        // Solve for t (& optionally s)
        double det = dx1 * dy2 - dx2 * dy1; // Find the determinant
        if (Math.abs(det) < 1e-10) {throw new RuntimeException("ERROR: RAYS ARE PARALLEL!");} // Stop divide by zero

        double t = (DX * dy2 - DY * dx2) / det;
        double s = (DX * dy1 - DY * dx1) / det; // Not technically required

        // Find the intersection of the 2 rays (A & B)
        if (t < 0 || s < 0) {throw new RuntimeException("ERROR: INVALID INTERSECTION");} // Stop invalid intersections

        TX = X1 + t * dx1;
        TY = Y1 + t * dy1;

        telemetry.addLine("Done Mathing!");
        telemetry.addData("TY", TY);
        telemetry.addData("TX", TX);
        telemetry.update();
    }


    public void CenterTag(int tagID) {
        // compute the midpoint of the camera frame
        final double centerX = CamW;
        AprilTagDetection target = null;

        // 1) Spin until we see the tag at all
        while (target == null) {
            for (AprilTagDetection det : tagProcessor.getDetections()) {
                if (det.id == tagID) {
                    target = det;
                    telemetry.addData("Tag Found", det.id);
                    telemetry.addData("Center X", det.center.x);
                    telemetry.update();
                    break;
                }
            }
            if (target == null) {
                robot.rotateRight(0.1);
                sleep(50);
            }
        }
        // stop spinning as soon as we’ve locked onto the tag
        robot.stopMotors();



        // 2) Center on the tag
        if (target != null) {
            double targetError;
            targetError = target.center.x - centerX;

            while (Math.abs(targetError) > 3) {
                while (Math.abs(targetError) > 3) {
                    // re-fetch detections each pass
                    target = null;
                    for (AprilTagDetection det : tagProcessor.getDetections()) {
                        if (det.id == tagID) {
                            target = det;
                            telemetry.addData("Center X", det.center.x);
                            break;
                        }
                    }
                    if (target == null) {
                        break;
                    }

                    // compute error relative to midpoint
                    targetError = target.center.x - centerX;
                    telemetry.addData("TargetError", targetError);
                    telemetry.update();

                    if (targetError > 3) {
                        robot.rotateRight(Math.max(Math.abs(targetError) / 1500, 0.05));
                    } else if (targetError < -3) {
                        robot.rotateLeft(Math.max(Math.abs(targetError) / 1500, 0.05));
                    }

                    sleep(25);
                }

                // finally, stop any motion
                robot.stopMotors();
            }
        }
    }

    public void Collect(String Color) {

    }


    public void ScoreBlock() {
        /*
        0.15 MAX AREA - Limelight

        Servo Start Pos:
            0.505 [Servo 1]
            0.470 [Servo 2]

        Lift Move Servo Pos:
            0.800 [Servo 1]
            0.505 [Servo 2]

        ------------------------------

         Servo Limits:
            Servo 1:
              0.63 MAX Right
              0.37 MAX Left

            Servo 2:
              0.35 MAX Down
              0.47 MAX Up
         */
    }


    public void HomePos() {
        // ToDo: GoTo (2, 2), then: slow strafe right for 3 sec, slow backwards for 3 sec.
        robot.goTo(0.8, "5", "5", "0");
        robot.driveBackward(0.2);
        sleep(3000);
        robot.strafeLeft(0.2);
        sleep(3000);
        robot.getImu().resetTracking();
    }
//endregion

    //region SparkFunOTOS Config
    @SuppressLint("DefaultLocale")
    private void configureOtos() {
        // SparkFun.setLinearUnit(DistanceUnit.METER);
        robot.getImu().setLinearUnit(DistanceUnit.INCH);
        // SparkFun.setAngularUnit(AngleUnit.RADIANS);
        robot.getImu().setAngularUnit(AngleUnit.DEGREES);

        robot.getImu().setLinearScalar(1.0);
        robot.getImu().setAngularScalar(1.0);

        robot.getImu().calibrateImu();

        robot.getImu().resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.getImu().setPosition(currentPosition);
    }
//endregion

}
