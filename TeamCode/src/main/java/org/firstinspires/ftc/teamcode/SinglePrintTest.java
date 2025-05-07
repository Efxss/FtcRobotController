package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@Autonomous
public class SinglePrintTest extends OpMode {
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        System.out.println("sout test (before sleep)");
        telemetry.log().add("telemetry log test (before sleep)");

        sleep(500);

        System.out.println("sout test (after sleep)");
        telemetry.log().add("telemetry log test (after sleep)");

        requestOpModeStop();
    }
}
