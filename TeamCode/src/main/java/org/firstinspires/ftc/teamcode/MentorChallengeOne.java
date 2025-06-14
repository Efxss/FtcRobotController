package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
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
    
// START GLOBAL VARIABLES-{
    
    // WebCam Globals:
        private static final double camWidthPX  = 640;
        private static final double camHeightPX = 480;
        private VisionPortal visionPortal;
        private AprilTagProcessor tagProcessor;
        
    // Generic [Robot] Globals:
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Trig [Triangulation] Globals:
        int basketRange = 5; // Inches
        float X1;
        float Y1;
        float H1;
        float X2;
        float Y2;
        float H2;
        float XT;
        float YT;

    // SparkFunOTOS Globals:
        SparkFunOTOS SparkFun;
        SparkFunOTOS.Pose2D pos;
        
//  }-END GLOBAL VARIABLES
    
// START INIT-{
    
    @Override
    public void mainInit() {

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tagProcessor);

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");
        SparkFun = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

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

        configureOtos();

// }-END INIT

// START MAIN SCRIPT-{

        // waitForStart(); // Incorrect Use? - Fix Please

        pos = SparkFun.getPosition();

        TriangulateBasketPos();

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

// }-END MAIN SCRIPT
    }

    @Override
    public void mainLoop() {
        // Nothing Here
    }

    
    public void TriangulateBasketPos() {
        MotorUtils.MoveDis(0.8, 12, SparkFun, frontLeft, frontRight, backLeft, backRight);
        MotorUtils.StrafeDis(0.8, 12, SparkFun, frontLeft, frontRight, backLeft, backRight);
        MotorUtils.RotateTo(0.8, 45, SparkFun, frontLeft, frontRight, backLeft, backRight);
        MotorUtils.StrafeDis(0.8, -6, SparkFun, frontLeft, frontRight, backLeft, backRight);

        pos = SparkFun.getPosition();
        X1 = (float) pos.x;
        Y1 = (float) pos.y;

        CenterTag(17);
        pos = SparkFun.getPosition();
        H1 = (float) Math.toRadians(pos.h);

        MotorUtils.RotateTo(0.8, 45, SparkFun, frontLeft, frontRight, backLeft, backRight);
        MotorUtils.StrafeDis(0.8, 12, SparkFun, frontLeft, frontRight, backLeft, backRight);

        pos = SparkFun.getPosition();
        X2 = (float) pos.x;
        Y2 = (float) pos.y;

        CenterTag(17);
        pos = SparkFun.getPosition();
        H2 = (float) Math.toRadians(pos.h);

        telemetry.addData("X1:", X1);
        telemetry.addData("Y1:", Y1);
        telemetry.addData("H1 (RAD):", "%.10f", H1);
        telemetry.addLine();

        telemetry.addData("X2:", X2);
        telemetry.addData("Y2:", Y2);
        telemetry.addData("H2 (RAD):", "%.10f", H2);
        telemetry.addLine();

        telemetry.addLine("Mathing...");
        telemetry.addLine(" -Be Aware: Could divide by 0 or <1e-6, very unlikely though-"); // POTENTIAL CATASTROPHIC ERROR: DIVIDE BY 0 OR <1e-6!!! IDK: How to fix/If even issue
        telemetry.update();

        XT = (float) ((Y2 - Y1 + (Math.tan(H1) * X1) - (Math.tan(H2) * X2)) / (Math.tan(H1) - Math.tan(H2)));
        YT = (float) (Math.tan(H1) * (XT - X1) + Y1);

        telemetry.addLine();
        telemetry.addLine("Done Mathing!");
        telemetry.addData("XT", XT);
        telemetry.addData("YT", YT);
    }

    
    public void CenterTag(int tagID) {
        double targetError = 10;

        while (targetError > 5) {
            AprilTagDetection target = null;

            while (target == null) {
                for (AprilTagDetection det : tagProcessor.getDetections()) {
                    if (det.id == tagID) {
                        target = det;
                        break;
                    }
                }
                MotorUtils.RotateTo(0.5, ((float) (pos.h + 15)), SparkFun, frontLeft, frontRight, backLeft, backRight);
            }
            targetError = target.center.x - (camWidthPX / 2.0);
            if (targetError > 0) {MotorUtils.RotateRight(0.5, frontLeft, frontRight, backLeft, backRight);}
            if (targetError < 0) {MotorUtils.RotateLeft(0.5, frontLeft, frontRight, backLeft, backRight);}
        }
        MotorUtils.StopMotors(frontLeft, frontRight, backLeft, backRight);
    }

    
    public void Collect(String Color) {

    }

    
    public void ScoreBlock() {

        
    }

    
    public void HomePos() {
        
    }
    
    
    @SuppressLint("DefaultLocale")
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // SparkFun.setLinearUnit(DistanceUnit.METER);
        SparkFun.setLinearUnit(DistanceUnit.INCH);
        // SparkFun.setAngularUnit(AngleUnit.RADIANS);
        SparkFun.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        //SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        //SparkFun.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        SparkFun.setLinearScalar(1.0);
        SparkFun.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        SparkFun.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        SparkFun.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        SparkFun.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        SparkFun.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}
