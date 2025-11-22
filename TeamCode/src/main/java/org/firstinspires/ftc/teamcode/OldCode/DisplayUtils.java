// !!!IMPORTANT!!! ::: THIS LIBRARY IS NOT FINISHED OR FULLY TESTED!!!

/**
    interrupt.err = {char(!!!IMPORTANT!!! ::: THIS LIBRARY IS NOT FINISHED OR FULLY TESTED!!!)};
    compiler.err(ERROR_INTERRUPT);
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utils.sleep;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

//@SuppressWarnings("unused") // If a function is not used in any other code, the compiler labels if as "unused"
public class DisplayUtils {

    //.init.*
    private static Telemetry SysTelemetry;
    private static Gamepad SysGamepad1 = new Gamepad();
    private static Gamepad SysGamepad2 = new Gamepad();

    //.gamepad.*
    public enum GamepadTarget {
        GAMEPAD1, GAMEPAD2, BOTH
    }
    public enum BlinkType {
        EVEN, ODD_HIGH, ODD_LOW
    }

    private static double lastGamepad1R = 0, lastGamepad1G = 0, lastGamepad1B = 0;
    private static double lastGamepad2R = 0, lastGamepad2G = 0, lastGamepad2B = 0;

    //.telemetry.*
    public enum ValueType {
        INT,
        DOUBLE,
        FLOAT,
        BOOLEAN
    }

    //endregion

    //region DisplayUtils.helpReference:
     /**
      * -----------------------------------For best readability, please expand this JavaDoc window so you can view this sentence on a single line-----------------------------------
      * <br>
      * <h1>DisplayUtils.java functions list and documentation key (helpReference)</h1>
      * <p>
      * <strong>Things to note:</strong><br>
      * You must call one or more of the following from your OpMode’s <code>mainInit()</code> method to use DisplayUtils features:
      * <code>DisplayUtils.init.initTelemetry(telemetry)</code>, <code>initGamepad1(gamepad1)</code>, or <code>initGamepad2(gamepad2)</code>.
      * </p>
      * <br>
      * <h3>DisplayUtils Documentation Key:</h3>
      * <p>
      * <strong>[Description]</strong><br>
      * [Notes/Notices]<br>
      * Dependencies (List)<br>
      * - Parameter(s)<br>
      * - Return Value (If any)
      * - Throws Value (If any)
      * </p>
      * <br>
      * <h3>DisplayUtils Functions List:</h3>
      * <strong>All Public DisplayUtils Functions:</strong>
      * <ul>
      *     <li><code>DisplayUtils.helpReference();</code></li>
      *     <br>
      *     <li><code>DisplayUtils.init.initTelemetry(telemetry);</code></li>
      *     <li><code>DisplayUtils.init.initGamepad1(gamepad1);</code></li>
      *     <li><code>DisplayUtils.init.initGamepad2(gamepad2);</code></li>
      *     <li><code>DisplayUtils.init.setTelemetryTransmissionRate(milliseconds);</code></li>
      *     <br>
      *     <li><code>DisplayUtils.gamepad.led.setLED(GamepadTarget, r, g, b, duration);</code></li>
      *     <li><code>DisplayUtils.gamepad.led.floatLED(GamepadTarget, r, g, b, speed, resolution);</code></li>
      *     <li><code>DisplayUtils.gamepad.led.sharpBlinkLED(GamepadTarget, r1, g1, b1, r2, g2, b2, speed, blinkType);</code></li>
      *     <li><code>DisplayUtils.gamepad.led.softPulseLED(GamepadTarget, r1, g1, b1, r2, g2, b2, speed, resolution, blinkType);</code></li>
      *     <li><code>DisplayUtils.gamepad.led.rainbowLED(GamepadTarget, speed, resolution);</code></li>
      *     <br>
      *     <li><code>DisplayUtils.gamepad.rumble.advRumble(GamepadTarget, rumbleLeft, rumbleRight, duration);</code></li>
      *     <br>
      *     <li><code>DisplayUtils.telemetry.menu.createMenu(menuID);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.removeMenu(menuID);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.addMenuItem(menuID, itemName);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.addMenuItem(menuID, itemName, itemVariable);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.addMenuItem(menuID, itemName, itemVariable, defaultVal);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.removeMenuItem(menuID, itemName);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.addMenuData(menuID, caption);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.addMenuData(menuID, caption, dataVariable);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.clearMenuData(menuID);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.displayMenu(menuID, gamepad);</code></li>
      *     <li><code>DisplayUtils.telemetry.menu.setOnMenuUpdate(menuID, menuID -> {...});</code></li>
      *     <li><code><b>[Object]</b> DisplayUtils.telemetry.menu.getMenuItemValue(menuID, itemName);</code></li>
      *     <br>
      *     <li><code>DisplayUtils.telemetry.log.setMaxLines(maxLines);</code></li>
      *     <li><code>DisplayUtils.telemetry.log.addLine(message);</code></li>
      *     <li><code>DisplayUtils.telemetry.log.throwSoftError(object, error, gamepadNotice);</code></li>
      *     <li><code>DisplayUtils.telemetry.log.throwHardError(object, error, safeShutdown);</code></li>
      *     <li><code>DisplayUtils.telemetry.log.clearLog(displayClearEvent);</code></li>
      * </ul>
      * <br>
      * <strong>All Internal DisplayUtils Functions: (For Debugging)</strong>
      * <ul>
      *     <li><code>Gamepad.LedEffect LEDSmoothTransition(r1, g1, b1, r2, g2, b2, speed, resolution).setRepeating(repeating).build();</code></li>
      *     <li><code>double interpolate(start, end, progress);</code></li>
      *     <br>
      *     <li><code>interface MenuUpdateListener;</code></li>
      *     <li><code>.setOnMenuUpdate(listener);</code></li>
      *     <li><code>.onMenuUpdate(menuID);</code></li>
      * </ul>
      * <br><br><br>
      */
    public static void helpReference() {}
    //endregion

    //region DisplayUtils.init.*
    public static class init {

        /**
         * <strong>Initializes the DisplayUtils telemetry instance.</strong><br>
         * <br>It is <strong>REQUIRED</strong> that you call this method BEFORE calling any other {@code DisplayUtils.*} methods!<br><br>
         * <strong>Dependencies:</strong> None
         * @param initTelemetry The {@link Telemetry} instance to be used by DisplayUtils.
         * @throws RuntimeException If {@code initTelemetry} is {@code null} (thrown via {@code .throwHardError()}).<br>
         */
        public static void initTelemetry(Telemetry initTelemetry) {
            if (initTelemetry == null) {
                telemetry.log.throwHardError("DisplayUtils.init.initTelemetry", "Telemetry retuned NULL!", false);
            } else {
                SysTelemetry = initTelemetry;
                SysTelemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
                SysTelemetry.log().clear();
            }
        }

        /**
         * <strong>Initializes the DisplayUtils gamepad1 instance.</strong><br>
         * <br>It is <strong>REQUIRED</strong> that you call this method BEFORE calling any other {@code DisplayUtils.gamepad.*} methods!<br><br>
         * <strong>Dependencies:</strong>
         * <ul>
         *     <li><code>initTelemetry();</code></li>
         * </ul>
         * @param initGamepad The first {@link Gamepad} instance to be used by DisplayUtils.
         * @throws android.os.strictmode.CustomViolation [Custom Error] If {@code initGamepad} is {@code null} or {@code synthetic} (thrown via {@code .throwSoftError()}).<br>
         */
        public static void initGamepad1(Gamepad initGamepad) { // Optional, Required for .gamepad (1) functions | REQUIRES Telemetry
            if (initGamepad.id == Gamepad.ID_UNASSOCIATED || initGamepad.id == Gamepad.ID_SYNTHETIC) {
                telemetry.log.throwSoftError("DisplayUtils.init.initGamepad1", "Gamepad1 is NOT connected to the device!", false);
            } else {
                SysGamepad1 = initGamepad;
                SysGamepad1.setLedColor(0, 0, 0, -1); // Sets to black as default
                lastGamepad1R = 0; lastGamepad1G = 0; lastGamepad1B = 0;
            }
        }

        /**
         * <strong>Initializes the DisplayUtils gamepad2 instance.</strong><br>
         * <br>It is <strong>REQUIRED</strong> that you call this method BEFORE calling any other {@code DisplayUtils.gamepad.*} methods!<br><br>
         * <strong>Dependencies:</strong>
         * <ul>
         *     <li><code>initTelemetry();</code></li>
         * </ul>
         * @param initGamepad The second {@link Gamepad} instance to be used by DisplayUtils.
         * @throws android.os.strictmode.CustomViolation [Custom Error] If {@code initGamepad} is {@code null} or {@code synthetic} (thrown via {@code .throwSoftError()}).<br>
         */
        public static void initGamepad2(Gamepad initGamepad) { // Optional, Required for .gamepad (2) functions
            if (initGamepad.id == Gamepad.ID_UNASSOCIATED || initGamepad.id == Gamepad.ID_SYNTHETIC) {
                telemetry.log.throwSoftError("DisplayUtils.init.initGamepad2", "Gamepad2 is NOT connected to the device!", false);
            } else {
                SysGamepad2 = initGamepad;
                SysGamepad2.setLedColor(0, 0, 0, -1); // Sets to black as default
                lastGamepad2R = 0; lastGamepad2G = 0; lastGamepad2B = 0;
            }
        }

        /**
         * <strong>Sets the DisplayUtils telemetry transmission rate.</strong><br>
         * <br>This method is <strong>NOT</strong> required. If this method is not called, the transmission rate default to 250ms.<br><br>
         * <strong>Dependencies:</strong> None
         * @param milliseconds The telemetry transmission rate, in ms, to be used by DisplayUtils.
         */
        public static void setTelemetryTransmissionRate(int milliseconds) {
            SysTelemetry.setMsTransmissionInterval(milliseconds);
        }

    }
    //endregion

    //region DisplayUtils.gamepad.*
    public static class gamepad {

        //region DisplayUtils.gamepad.led.*
        public static class led {

            // ToDo: Note: Set Duration to -1 for inf
            /**
             * <strong>Sets the target gamepad</strong><br>
             * <br><br><br>
             * <strong>Dependencies:</strong><br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param
             * @return
             * @throws
             */
            public static void setLED(GamepadTarget Gamepad, double r, double g, double b, int Duration) {
                if (Gamepad == GamepadTarget.GAMEPAD1 || Gamepad == GamepadTarget.BOTH) {
                    SysGamepad1.setLedColor(r, g, b, Duration);
                    if (Duration == -1) {lastGamepad1R = r; lastGamepad1G = g; lastGamepad1B = b;}
                }
                if (Gamepad == GamepadTarget.GAMEPAD2 || Gamepad == GamepadTarget.BOTH) {
                    SysGamepad2.setLedColor(r, g, b, Duration);
                    if (Duration == -1) {lastGamepad2R = r; lastGamepad2G = g; lastGamepad2B = b;}
                }
            }

            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void floatLED(GamepadTarget Gamepad, double r, double g, double b, int speed, int resolution) { // TODO: Add a notice: Speed/Res = MS per step, recommended: 10 - 40 ms per step (usually Res = Speed/10 is good)
                if (Gamepad == GamepadTarget.GAMEPAD1 || Gamepad == GamepadTarget.BOTH) {
                    Gamepad.LedEffect GP1_Effect = LEDSmoothTransition(lastGamepad1R, lastGamepad1G, lastGamepad1B, r, g, b, speed, resolution).setRepeating(false).build();
                    SysGamepad1.runLedEffect(GP1_Effect);
                    lastGamepad1R = r; lastGamepad1G = g; lastGamepad1B = b;
                }

                if (Gamepad == GamepadTarget.GAMEPAD2 || Gamepad == GamepadTarget.BOTH) {
                    Gamepad.LedEffect GP2_Effect = LEDSmoothTransition(lastGamepad2R, lastGamepad2G, lastGamepad2B, r, g, b, speed, resolution).setRepeating(false).build();
                    SysGamepad2.runLedEffect(GP2_Effect);
                    lastGamepad2R = r; lastGamepad2G = g; lastGamepad2B = b;
                }
            }

            // ToDo: Note: less precise bcs of required double to int conversion
            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void sharpBlinkLED(GamepadTarget Gamepad, double r1, double g1, double b1, double r2, double g2, double b2 , int speed, BlinkType blinkType) {
                double onDuration = speed * 0.50, offDuration = speed * 0.50;
                if (blinkType == DisplayUtils.BlinkType.EVEN)        {onDuration = speed * 0.50; offDuration = speed * 0.50;}
                if (blinkType == DisplayUtils.BlinkType.ODD_HIGH)    {onDuration = speed * 0.75; offDuration = speed * 0.25;}
                if (blinkType == DisplayUtils.BlinkType.ODD_LOW)     {onDuration = speed * 0.25; offDuration = speed * 0.75;}

                Gamepad.LedEffect.Builder SBLED_Builder = new Gamepad.LedEffect.Builder();

                SBLED_Builder.addStep(r1, g1, b1, (int) onDuration);
                SBLED_Builder.addStep(r2, g2, b2, (int) offDuration);

                if (Gamepad == GamepadTarget.GAMEPAD1 || Gamepad == GamepadTarget.BOTH) {
                    Gamepad.LedEffect GP1_Effect = SBLED_Builder.setRepeating(true).build();
                    SysGamepad1.runLedEffect(GP1_Effect);
                }
                if (Gamepad == GamepadTarget.GAMEPAD2 || Gamepad == GamepadTarget.BOTH) {
                    Gamepad.LedEffect GP2_Effect = SBLED_Builder.setRepeating(true).build();
                    SysGamepad2.runLedEffect(GP2_Effect);
                }
            }

            // @throws IllegalArgumentException if ...
            // ToDo: Note: less precise bcs of required double to int conversion
            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void softPulseLED(GamepadTarget Gamepad, double r1, double g1, double b1, double r2, double g2, double b2, int speed, int resolution, BlinkType blinkType) {
                if (resolution <= 0) {throw new IllegalArgumentException("[DisplayUtils.gamepad.led.advBlinkLED]: <ERROR> When calculating smooth LED transition, precation caught DIVIDE BY ZERO (Variable: 'resolution' <= 0)!");}

                int step;

                double onDuration = speed * 0.50, offDuration = speed * 0.50;
                if (blinkType == DisplayUtils.BlinkType.EVEN)        {onDuration = speed * 0.50; offDuration = speed * 0.50;}
                if (blinkType == DisplayUtils.BlinkType.ODD_HIGH)    {onDuration = speed * 0.75; offDuration = speed * 0.25;}
                if (blinkType == DisplayUtils.BlinkType.ODD_LOW)     {onDuration = speed * 0.25; offDuration = speed * 0.75;}

                int res = resolution / 2;
                step = (int) onDuration / res;

                Gamepad.LedEffect.Builder SPLED_Builder = new Gamepad.LedEffect.Builder();
                double currentR = 0, currentG = 0, currentB = 0;

                for (int i = 0; i <= res + 1; i++) {
                    double progress = (double) i / res;

                    currentR = interpolate(r2, r1, progress);
                    currentG = interpolate(g2, g1, progress);
                    currentB = interpolate(b2, b1, progress);
                    SPLED_Builder.addStep(currentR, currentG, currentB, step);
                }

                step = (int) offDuration / res;

                for (int i = 0; i <= res + 1; i++) {
                    double progress = (double) i / res;

                    currentR = interpolate(r1, r2, progress);
                    currentG = interpolate(g1, g2, progress);
                    currentB = interpolate(b1, b2, progress);
                    SPLED_Builder.addStep(currentR, currentG, currentB, step);
                }
                SPLED_Builder.addStep(currentR, currentG, currentB, step * 5);

                if (Gamepad == GamepadTarget.GAMEPAD1 || Gamepad == GamepadTarget.BOTH) {
                    Gamepad.LedEffect GP1_Effect = SPLED_Builder.setRepeating(true).build();
                    SysGamepad1.runLedEffect(GP1_Effect);
                }
                if (Gamepad == GamepadTarget.GAMEPAD2 || Gamepad == GamepadTarget.BOTH) {
                    Gamepad.LedEffect GP2_Effect = SPLED_Builder.setRepeating(true).build();
                    SysGamepad2.runLedEffect(GP2_Effect);
                }
            }

            // @throws IllegalArgumentException if ...
            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void rainbowLED(GamepadTarget Gamepad, int speed, int resolution) {
                if (resolution <= 0) {throw new IllegalArgumentException("[DisplayUtils.gamepad.led.rainbowLED]: <ERROR> When calculating smooth LED transition, precation caught DIVIDE BY ZERO (Variable: 'resolution' <= 0)!");}

                int res = resolution / 3;
                int step = (speed / 3) / res;

                Gamepad.LedEffect.Builder RBLED_Builder = new Gamepad.LedEffect.Builder();
                double currentR = 0, currentG = 0, currentB = 0;

                for (int i = 0; i <= res + 1; i++) {
                    double progress = (double) i / res;

                    currentR = interpolate(0, 1, progress);
                    currentG = interpolate(0, 0, progress);
                    currentB = interpolate(1, 0, progress);
                    RBLED_Builder.addStep(currentR, currentG, currentB, step);
                }
                for (int i = 0; i <= res + 1; i++) {
                    double progress = (double) i / res;

                    currentR = interpolate(1, 0, progress);
                    currentG = interpolate(0, 1, progress);
                    currentB = interpolate(0, 0, progress);
                    RBLED_Builder.addStep(currentR, currentG, currentB, step);
                }
                for (int i = 0; i <= res + 1; i++) {
                    double progress = (double) i / res;

                    currentR = interpolate(0, 0, progress);
                    currentG = interpolate(1, 0, progress);
                    currentB = interpolate(0, 1, progress);
                    RBLED_Builder.addStep(currentR, currentG, currentB, step);
                }
                RBLED_Builder.addStep(currentR, currentG, currentB, step * 5);

                if (Gamepad == GamepadTarget.GAMEPAD1 || Gamepad == GamepadTarget.BOTH) {
                    Gamepad.LedEffect GP1_Effect = RBLED_Builder.setRepeating(true).build();
                    SysGamepad1.runLedEffect(GP1_Effect);
                }
                if (Gamepad == GamepadTarget.GAMEPAD2 || Gamepad == GamepadTarget.BOTH) {
                    Gamepad.LedEffect GP2_Effect = RBLED_Builder.setRepeating(true).build();
                    SysGamepad2.runLedEffect(GP2_Effect);
                }
            }

            private static Gamepad.LedEffect.Builder LEDSmoothTransition(double r1, double g1, double b1, double r2, double g2, double b2, int speed, int resolution) {
                if (resolution <= 0) {throw new IllegalArgumentException("[DisplayUtils.gamepad.led.LEDSmoothTransition]: <ERROR> When calculating smooth LED transition, precation caught DIVIDE BY ZERO (Variable: 'resolution' <= 0)!");}

                int step = speed / resolution;

                Gamepad.LedEffect.Builder LEDST_Builder = new Gamepad.LedEffect.Builder();

                for (int i = 0; i <= resolution + 1; i++) {
                    double progress = (double) i / resolution;

                    double currentR = interpolate(r1, r2, progress);
                    double currentG = interpolate(g1, g2, progress);
                    double currentB = interpolate(b1, b2, progress);
                    LEDST_Builder.addStep(currentR, currentG, currentB, step);
                }
                LEDST_Builder.addStep(r2, g2, b2, Integer.MAX_VALUE);

                return LEDST_Builder;
            }

            private static double interpolate(double start, double end, double progress) {
                return Math.max(0.0, Math.min(1.0, start + (end - start) * progress));
            }
        }
        //endregion

        //region DisplayUtils.gamepad.rumble.*
        public static class rumble {

            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void advRumble(GamepadTarget Gamepad, double rumbleLeft, double rumbleRight, int duration) {
                if (Gamepad == GamepadTarget.GAMEPAD1 || Gamepad == GamepadTarget.BOTH) {
                    DisplayUtils.SysGamepad1.rumble(rumbleLeft, rumbleRight, duration);}
                if (Gamepad == GamepadTarget.GAMEPAD2 || Gamepad == GamepadTarget.BOTH) {
                    DisplayUtils.SysGamepad2.rumble(rumbleLeft, rumbleRight, duration);}
            }
        }
        //endregion
    }
    //endregion

    //region DisplayUtils.telemetry.*
    public static class telemetry {

        //region DisplayUtils.telemetry.unstable.*
        public static class unstable { // TODO: REMOVE AFTER TESTING!
            //region DisplayUtils.telemetry.unstable.menu.*

            /** Status: 95%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
            public static class menu {
                private static final Map<String, Menu> menus = new HashMap<>();

                // Define interface for update callback

                /** Status: 99%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public interface MenuUpdateListener {
                    void onMenuUpdate(String menuID);
                }

                /** @noinspection FieldMayBeFinal*/ // To suppress warnings in 'menuID', 'items', & 'data'
                private static class Menu {
                    private String menuID;
                    private List<MenuItems> items;
                    private List<MenuData> data;

                    private MenuUpdateListener updateListener;

                    private Menu(String menuID, List<MenuItems> items, List<MenuData> data) {
                        this.menuID = menuID;
                        this.items = items;
                        this.data = data;
                    }

                    // Setter for update listener
                    private void setOnMenuUpdate(MenuUpdateListener listener) {
                        this.updateListener = listener;
                    }

                    // Call the listener if present
                    private void onMenuUpdate() {
                        if (updateListener != null) {
                            updateListener.onMenuUpdate(menuID);
                        }
                    }
                }

                /** @noinspection FieldMayBeFinal*/ // To suppress warnings in 'name', 'type', & 'defaultValue'
                private static class MenuItems {
                    private String name;
                    private ValueType type;
                    private Object variable;
                    private Object defaultValue;

                    private MenuItems(String name, ValueType type, Object variable, Object defaultValue) {
                        this.name = name;
                        this.type = type;
                        this.variable = variable;
                        this.defaultValue = defaultValue;
                    }
                }

                /** @noinspection FieldMayBeFinal*/ // To suppress warnings in 'caption' & 'value'
                private static class MenuData {
                    private String caption;
                    private Object value;

                    private MenuData(String caption, Object value) {
                        this.caption = caption;
                        this.value = value;
                    }
                }

                /** Status: 99%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void createMenu(String menuID) {
                    menus.put(menuID, new Menu(menuID, new ArrayList<>(), new ArrayList<>()));
                }

                // New method to set the listener from outside

                /** Status: 95%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void setOnMenuUpdate(String menuID, MenuUpdateListener listener) {
                    Menu menu = menus.get(menuID);
                    if (menu != null) {
                        menu.setOnMenuUpdate(listener);
                    }
                }

                /** Status: 99%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void removeMenu(String menuID) {
                    menus.remove(menuID);
                }

                /** Status: 96%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void addMenuItem(String menuID, String name) {
                    addMenuItem(menuID, name, null, null);
                }

                /** Status: 96%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void addMenuItem(String menuID, String name, Object variable) {
                    addMenuItem(menuID, name, variable, null);
                }

                /** Status: 96%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void addMenuItem(String menuID, String name, Object variable, Object defaultValue) {
                    Menu menu = menus.get(menuID);
                    if (menu == null) return;

                    if (variable instanceof Integer) {
                        menu.items.add(new MenuItems(name, ValueType.INT, variable, defaultValue));
                    } else if (variable instanceof Float) {
                        menu.items.add(new MenuItems(name, ValueType.FLOAT, variable, defaultValue));
                    } else if (variable instanceof Double) {
                        menu.items.add(new MenuItems(name, ValueType.DOUBLE, variable, defaultValue));
                    } else if (variable instanceof Boolean) {
                        menu.items.add(new MenuItems(name, ValueType.BOOLEAN, variable, defaultValue));
                    } else {
                        menu.items.add(new MenuItems(name, null, variable, defaultValue));
                    }
                }

                /** Status: 98%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void removeMenuItem(String menuID, String itemName) {
                    Menu menu = menus.get(menuID);
                    if (menu != null) {
                        menu.items.removeIf(item -> item.name.equals(itemName));
                    }
                }

                /** Status: 98%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void addMenuData(String menuID, String caption) {
                    Menu menu = menus.get(menuID);
                    if (menu != null) {
                        menu.data.add(new MenuData(caption, null));
                    }
                }

                /** Status: 98%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void addMenuData(String menuID, String caption, Object variable) {
                    Menu menu = menus.get(menuID);
                    if (menu != null) {
                        menu.data.add(new MenuData(caption, variable));
                    }
                }

                /** Status: 99%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void clearMenuData(String menuID) {
                    Menu menu = menus.get(menuID);
                    if (menu != null) {
                        menu.data.clear();
                    }
                }

                /** Status: 95%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static Object getMenuItemValue(String menuID, String itemName) {
                    Menu menu = menus.get(menuID);
                    if (menu == null) return null;

                    for (MenuItems item : menu.items) {
                        if (item.name.equals(itemName)) {
                            return item.variable;
                        }
                    }
                    return null;
                }

                /** Status: 95%! - To Be Tested!*/ // TODO: REMOVE AFTER TESTING!
                public static void displayMenu(String menuID, Gamepad gamepad) {
                    Menu menu = menus.get(menuID);
                    if (menu == null) {
                        log.throwSoftError("DisplayUtils.telemetry.menu.displayMenu()", "Menu [ID]" + menuID + " does NOT exist!", true);
                        return;
                    }

                    log.addLine("Entered Menu [ID]" + menuID);

                    Gamepad selectorGamepad = new Gamepad();
                    int selectedItem = 0;
                    boolean editing = false;

                    boolean exitSelected = false;
                    boolean hasExit = false;
                    for (int i = 0; i < menu.items.size(); i++) {
                        MenuItems item = menu.items.get(i);
                        if (Objects.equals(item.name, "EXIT")) {
                            hasExit = true;
                        }
                    }
                    if (!hasExit) {
                        addMenuItem(menuID, "EXIT");
                    }

                    while (!exitSelected) {
                        SysTelemetry.clearAll();
                        SysTelemetry.addLine(menuID + "\n");

                        for (int i = 0; i < menu.items.size(); i++) {
                            MenuItems item = menu.items.get(i);
                            String selector = (selectedItem == i) ? (editing ? ">>" : "> ") : "  ";

                            SysTelemetry.addLine(selector + item.name + " : " + item.variable + " (Default: " + item.defaultValue + ")");
                        }

                        if (menu.data != null && !menu.data.isEmpty()) {
                            SysTelemetry.addLine("\n\nOUTPUT:\n");

                            for (int i = 0; i < menu.data.size(); i++) {
                                MenuData data = menu.data.get(i);

                                SysTelemetry.addLine(data.caption + " : " + data.value);
                            }
                        }

                        selectorGamepad.copy(gamepad);

                        if (!editing) {
                            if (selectorGamepad.dpad_up) {
                                sleep(100);
                                selectedItem = Math.min(Math.max(selectedItem - 1, 0), menu.items.size() - 1);
                            }
                            if (selectorGamepad.dpad_down) {
                                sleep(100);
                                selectedItem = Math.min(Math.max(selectedItem + 1, 0), menu.items.size() - 1);
                            }
                            if (selectorGamepad.dpad_right) {
                                sleep(100);
                                MenuItems item = menu.items.get(selectedItem);
                                if (!Objects.equals(item.name, "EXIT")) {
                                    editing = true;
                                } else {
                                    exitSelected = true;
                                }
                            }
                        } else {
                            MenuItems item = menu.items.get(selectedItem);

                            if (selectorGamepad.dpad_up) {
                                sleep(100);
                                switch (item.type) {
                                    case INT:
                                        item.variable = (Integer) item.variable + 5;
                                        break;
                                    case FLOAT:
                                        item.variable = (Float) item.variable + 0.5;
                                        break;
                                    case DOUBLE:
                                        item.variable = (Double) item.variable + 0.5;
                                        break;
                                    case BOOLEAN:
                                        item.variable = true;
                                        break;
                                    default:
                                        break;
                                }
                            }
                            if (selectorGamepad.dpad_down) {
                                sleep(100);
                                switch (item.type) {
                                    case INT:
                                        item.variable = (Integer) item.variable - 5;
                                        break;
                                    case FLOAT:
                                        item.variable = (Float) item.variable - 0.5;
                                        break;
                                    case DOUBLE:
                                        item.variable = (Double) item.variable - 0.5;
                                        break;
                                    case BOOLEAN:
                                        item.variable = false;
                                        break;
                                    default:
                                        break;
                                }
                            }
                            if (selectorGamepad.dpad_left) {
                                sleep(100);
                                editing = false;
                            }
                        }

                        menu.onMenuUpdate();

                        SysTelemetry.update();
                        sleep(80);
                    }
                    SysTelemetry.clearAll();
                    log.addLine("Exited Menu [ID]" + menuID);
                }
            }
            //endregion
        }
        //endregion

        //region DisplayUtils.telemetry.log.*
        public static class log {

            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void setMaxLines(int maxLines) {
                SysTelemetry.log().setCapacity(maxLines);
            }

            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void addLine(String message) {
                SysTelemetry.log().add(message);
            }

            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void throwSoftError(String object, String error, boolean gamepadNotice) {
                addLine("[" + object + "]: <ERROR> [SOFT] " + error);

                if (gamepadNotice) {
                    gamepad.rumble.advRumble(GamepadTarget.BOTH, 0.3, 0.3, 1000);
                    gamepad.led.sharpBlinkLED(GamepadTarget.BOTH, 1, 0, 0, 0, 0, 0, 1000, BlinkType.EVEN);
                }
            }

            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void throwHardError(String object, String error, boolean safeShutdown) {
                addLine("[" + object + "] <ERROR> [HARD] " + error);

                if (safeShutdown) {
                    addLine("SafeShutdown enabled, shutting down...");
                    ThreadOpMode.activeInstance.demandOpModeStop();
                } else {
                    addLine("SafeShutdown disabled, throwing runtime exception...");
                    throw new RuntimeException("[" + object + "] CUSTOM ERROR [HARD] " + error);
                }
            }

            /**
             * <strong>[Short Explanation]</strong><br>
             * [Expanded Description]<br>
             * Dependencies:<br>
             * <ul>
             *     <li></li>
             * </ul>
             * @param <> [Explanation]
             */
            public static void clearLog(boolean displayClearEvent) {
                if (displayClearEvent) {addLine("[Clearing Log...]");}
                SysTelemetry.log().clear();

                if (displayClearEvent) {addLine("[Log Cleared]");}
                addLine(" ");
            }
        }
        //endregion
    }
    //endregion
}

**/
