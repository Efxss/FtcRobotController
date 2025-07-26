package org.firstinspires.ftc.teamcode.threadopmode;
import static org.firstinspires.ftc.teamcode.Utils.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * A type of {@link OpMode} that contains threads to be ran in parallel periodically.
 * Register threads with {@link ThreadOpMode#registerThread(TaskThread)}
 */
public abstract class ThreadOpMode extends OpMode {
    private List<TaskThread> threads = new ArrayList<>();

    private boolean shutdownReady = false;
    private boolean autoShutdownRequested = false;
    /**
     * Registers a new {@link TaskThread} to be ran periodically.
     * Registered threads will automatically be started during {@link OpMode#start()} and stopped during {@link OpMode#stop()}.
     *
     * @param taskThread A {@link TaskThread} object to be ran periodically.
     */
    public final void registerThread(TaskThread taskThread) {threads.add(taskThread);}

    /**
     * Contains code to be ran before the OpMode is started. Similar to {@link OpMode#init()}.
     */
    public abstract void mainInit();
    /**
     * Contains code to be ran periodically in the MAIN thread. Similar to {@link OpMode#loop()}.
     */
    public abstract void mainLoop();

    /**
     * Should not be called by subclass.
     */
    @Override
    public final void init() {
        // Reset Shutdown Flags
        shutdownReady = false;
        autoShutdownRequested = false;
        // Run Main Init
        mainInit();
    }

    /**
     * Should not be called by subclass.
     */
    @Override
    public final void start() {
        for(TaskThread taskThread : threads) {
            taskThread.start();
        }
    }

    /**
     * Should not be called by subclass.
     */
    @Override
    public final void loop() {
        // Basically IF(StopRequested) --> Safe Shutdown
        if (gamepad1.guide || autoShutdownRequested) {
            if (shutdownReady) {requestOpModeStop();} else {
                gamepad1.rumble(0.5, 0.5, 1000);
                manualOpModStop();
            }
        } else {
            // Run Main Loop
            mainLoop();
        }
    }

    /**
     * Should not be called by subclass.
     */
    @Override
    public final void stop() {
        for(TaskThread taskThread : threads) {
            taskThread.stop();
        }
    }

    // Helpful functions:

    /**
     * Contains code to be ran before the OpMode is ended.
     */
    protected void onOpModeStop() {}

    /**
     * Should not be called by subclass.
     */
    protected void manualOpModStop() {
        onOpModeStop();
        sleep(1000);
        shutdownReady = true;
    }

    /**
     * Call to automatically enable safeShutdown without needing a gamepad input.
     */
    protected void requestAutoOpModeStop() {
        autoShutdownRequested = true;
    }

}
