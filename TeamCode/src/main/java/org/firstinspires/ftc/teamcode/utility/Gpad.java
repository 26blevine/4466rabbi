package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Locale;

/**
 * <h3>A class to manage {@link Gamepad}.</h3>
 * <p>Adds useful utils and normal Gamepad functions</p>
 * <br>
 * <h2>Main use:</h2>
 * Calculates whether a button should seem pressed. Used to make buttons only seem pressed after initial press, not while holding down.
 * <br>
 * <br>
 * <b>Note:</b> You must run {@link #update()} in your game loop to have this work. It updates all the button values
 * <br>
 * <br>

 * @author natanmeyer
 * @see Gamepad
 */
public class Gpad {
    private final Gamepad g;
    /** @VALUE X Button. Square on dual shock */
    public boolean
            x,
    /** @VALUE Y Button. Triangle on dual shock*/
    y,
    /** @VALUE A Button. Cross (X) on dual shock */
    a,
    /** @VALUE B Button. Circle on dual shock*/
    b,
    /** @VALUE Bumper Left*/
    bL,
    /** @VALUE Bumper Right*/
    bR,
    /** @VALUE Dpad Up*/
    dU,
    /** @VALUE Dpad Left*/
    dL,
    /** @VALUE Dpad Right*/
    dR,
    /** @VALUE Dpad Down*/
    dD,
    /** @VALUE Start Button*/
    start,
    /** @VALUE Back Button*/
    back,
    /** @VALUE Left Stick Button (press down on left stick)*/
    LStickButton,
    /** @VALUE Right Stick Button (press down on right stick)*/
    RStickButton,
    /** @VALUE If the touchpad is clicked*/
    t,
    /** @VALUE Options Button*/
    options,
    /** @VALUE Share Button*/
    share,
    /** @VALUE PS Button*/
    ps,
    /** @VALUE Guide Button*/
    guide=false;
    /** @VALUE Square Button. X on normal controller*/
    public boolean
            square,
    /** @VALUE Cross Button (it looks like an X). A on normal controller*/
    cross,
    /** @VALUE Triangle Button. Y on normal controller*/
    triangle,
    /** @VALUE Circle Button. B on normal controller*/
    circle,
    /** @VALUE Touchpad Finger One. If there's one finger on the touchpad*/
    t1,
    /** @VALUE Touchpad Finger Two. If there's a second finger on the touchpad*/
    t2,
    /** @VALUE Touchpad Finger One Press. Use as touchpad button. Tap on touchpad as button, not click **/
    t1Press,
    /** @VALUE Touchpad Finger Two Press. Use as touchpad button. Tap on touchpad as button, not click **/
    t2Press,
    /** @VALUE Touchpad Top Left Button. Represents the button if you click on the very top left of the touchpad
     *  @apiNote activated when {@link Gamepad#touchpad} is triggered and {@link Gamepad#touchpad_finger_1} is within (.2,.2) from the target corner**/
    tTopLeft,
    /** @VALUE Touchpad Top Right Button. Represents the button if you click on the very top right of the touchpad
     * @apiNote activated when {@link Gamepad#touchpad} is triggered and {@link Gamepad#touchpad_finger_1} is within (.2,.2) from the target corner**/
    tTopRight,
    /** @VALUE Touchpad Bottom Left Button. Represents the button if you click on the very bottom left of the touchpad
     * @apiNote activated when {@link Gamepad#touchpad} is triggered and {@link Gamepad#touchpad_finger_1} is within (.2,.2) from the target corner**/
    tBotLeft,
    /** @VALUE Touchpad Bottom Right Button. Represents the button if you click on the very bottom right of the touchpad
     * @apiNote activated when {@link Gamepad#touchpad} is triggered and {@link Gamepad#touchpad_finger_1} is within (.2,.2) from the target corner**/
    tBotRight,
    /** @VALUE Touchpad Top Left Press Button. Represents the button if you press on the very top left of the touchpad
     *  @apiNote activated when a touchpad finger is triggered and within (.2,.2) from the target corner**/
    tTopLeftPress,
    /** @VALUE Touchpad Top Right Press Button. Represents the button if you press on the very top right of the touchpad
     * @apiNote activated when a touchpad finger is triggered and within (.2,.2) from the target corner**/
    tTopRightPress,
    /** @VALUE Touchpad Bottom Left Press Button. Represents the button if you press on the very bottom left of the touchpad
     * @apiNote activated when a touchpad finger is triggered and within (.2,.2) from the target corner**/
    tBotLeftPress,
    /** @VALUE Touchpad Bottom Right Press Button. Represents the button if you press on the very bottom right of the touchpad
     * @apiNote activated when a touchpad finger is triggered and within (.2,.2) from the target corner**/
    tBotRightPress= false;

    private final int[] v = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //0 = not pressed, 1=pressed, 2=pressed waiting to be unpressed

    /** @VALUE Touchpad Finger One X Position (0-1)*/
    public float
            t1x,
    /** @VALUE Touchpad Finger One Y Position (0-1)*/
    t1y,
    /** @VALUE Touchpad Finger Two X Position (0-1)*/
    t2x,
    /** @VALUE Touchpad Finger Two Y Position (0-1)*/
    t2y,
    /** @VALUE Right Stick X Position (0-1) */
    stickRX,
    /** @VALUE Right Stick Y Position (0-1) */
    stickRY,
    /** @VALUE Left Stick X Position (0-1) */
    stickLX,
    /** @VALUE Left Stick Y Position (0-1) */
    stickLY,
    /** @VALUE Right Stick Trigger Value (0-1) */
    RTrigger,
    /** @VALUE Left Stick Trigger Value (0-1) */
    LTrigger;

    public Gpad(Gamepad gamepad) {
        this.g = gamepad;
    }

    /** Updates values of all buttons using either:
     * <li> raw values</li>
     * <li>calculated values to make buttons only true after initial press, not while holding down</li>
     * @see #eval(boolean press, int index)
     */
    public void update(){
        x=eval(g.x,0);
        y=eval(g.y,1);
        a=eval(g.a,2);
        b=eval(g.b,3);
        start=eval(g.start,4);
        bL=eval(g.left_bumper,5);
        bR=eval(g.right_bumper,6);
        dU=eval(g.dpad_up,7);
        dL=eval(g.dpad_left,8);
        dR=eval(g.dpad_right,9);
        dD=eval(g.dpad_down,10);
        back=eval(g.back,11);
        LStickButton =eval(g.left_stick_button,12);
        RStickButton =eval(g.right_stick_button,13);
        t = eval(g.touchpad, 14);
        // these are normal b/c they aren't buttons. only other case is using t1 or t2 as touchpad to just press, not click
        t1 = g.touchpad_finger_1;
        t2 = g.touchpad_finger_2;
        t1x = g.touchpad_finger_1_x;
        t2x = g.touchpad_finger_2_x;
        t1y = g.touchpad_finger_1_y;
        t2y = g.touchpad_finger_2_y;

        // finger mainly matters for null errors or if pressing it somehow doesn't trigger finger pos (gloves?)
        // use o.touchpad so that you get the right value, not the one in this class
        if(!g.touchpad_finger_1 || !g.touchpad){
            tTopLeft=eval(false, 15);
            tTopRight=eval(false, 16);
            tBotLeft=eval(false, 17);
            tBotRight=eval(false, 18);
        } else {
            // if here, it means the touchpad was clicked and the finger is on the touchpad
            // if finger in correct position for button, then button is pressed!
            tTopLeft = eval(t1x < -0.8 && t1y > 0.8,15);
            tTopRight = eval(t1x > 0.8 && t1y > 0.8,16);
            tBotLeft = eval(t1x < -0.8 && t1y < -0.8,17);
            tBotRight = eval(t1x > 0.8 && t1y < -0.8,18);
        }

        if((!g.touchpad_finger_1 && !g.touchpad_finger_2) || g.touchpad){
            tTopLeftPress=eval(false, 19);
            tTopRightPress=eval(false, 20);
            tBotLeftPress=eval(false, 21);
            tBotRightPress=eval(false, 22);
        } else {
            if(g.touchpad_finger_1 && g.touchpad_finger_2){
                if(t1x < -0.8 && t1y > 0.8 || t2x < -0.8 && t2y > 0.8){
                    tTopLeftPress = eval(true,19);
                }
                if(t1x > 0.8 && t1y > 0.8 || t2x > 0.8 && t2y > 0.8){
                    tTopRightPress = eval(true,20);
                }
                if(t1x < -0.8 && t1y < -0.8 || t2x < -0.8 && t2y < -0.8){
                    tBotLeftPress = eval(true,21);
                }
                if(t1x > 0.8 && t1y < -0.8 || t2x > 0.8 && t2y < -0.8){
                    tBotRightPress = eval(true,22);
                }
            } else if (g.touchpad_finger_1){
                tTopLeftPress = eval(t1x < -0.8 && t1y > 0.8,19);
                tTopRightPress = eval(t1x > 0.8 && t1y > 0.8,20);
                tBotLeftPress = eval(t1x < -0.8 && t1y < -0.8,21);
                tBotRightPress = eval(t1x > 0.8 && t1y < -0.8,22);
            } else if(g.touchpad_finger_2){
                tTopLeftPress = eval(t2x < -0.8 && t2y > 0.8,19);
                tTopRightPress = eval(t2x > 0.8 && t2y > 0.8,20);
                tBotLeftPress = eval(t2x < -0.8 && t2y < -0.8,21);
                tBotRightPress = eval(t2x > 0.8 && t2y < -0.8,22);
            }
        }

        t1Press = eval(g.touchpad_finger_1, 23);
        t2Press = eval(g.touchpad_finger_2, 24);

        // set doubled vars or vars that are always raw values (sticks, triggers, etc)
        circle = b;
        cross = a;
        triangle = y;
        square = x;
        options = start;
        share = back;
        ps = guide;
        stickRX = g.right_stick_x;
        stickRY = g.right_stick_y;
        stickLX = g.left_stick_x;
        stickLY = g.left_stick_y;
        stickRX = g.right_stick_x;
        RTrigger = g.right_trigger;
        LTrigger = g.left_trigger;


    }

    /**
     * Calculates whether a button should seem pressed. Used to make buttons only seem pressed after initial press, not while holding down.
     * @param pressed If the raw value of the button is currently pressed
     * @param i index in {@link #v}
     * @return if the button should seem pressed to the user
     */
    private boolean eval(boolean pressed, int i){
        if(pressed){
            if(v[i]==0){
                v[i]=1;
                return true;
            } else return v[i] != 1;
        }else{
            v[i]=0;
            return false;
        }
    }

    /**
     * Rumble the gamepad's first rumble motor at maximum power for a certain duration.
     * Calling this will displace any currently running rumble effect.
     * @param duration milliseconds to rumble for, or {@link Gamepad#RUMBLE_DURATION_CONTINUOUS}
     */
    public void rumble(int duration){
        g.rumble(duration);
    }

    /**
     * Rumble the gamepad at a fixed rumble power for a certain duration
     * Calling this will displace any currently running rumble effect
     * @param power rumble power (0.0 - 1.0)
     * @param duration milliseconds to rumble for,  or {@link Gamepad#RUMBLE_DURATION_CONTINUOUS}
     */
    public void rumble(double power, int duration){
        g.rumble(power, power, duration);
    }

    /**
     * Rumble the gamepad at a fixed rumble power
     * Calling this will displace any currently running rumble effect
     * @param power rumble power (0.0 - 1.0)
     */
    public void rumble(double power){
        g.rumble(power, power, Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    /**
     * @return An educated guess about whether there is a rumble action ongoing on this gamepad
     */
    public boolean isRumbling(){
        return g.isRumbling();
    }

    /**
     * Cancel the currently running rumble effect, if any
     */
    public void stopRumble(){
        g.stopRumble();
    }

    /**
     * Rumble the gamepad for a certain number of "blips" using predetermined blip timing
     * This will displace any currently running rumble effect.
     * @param count the number of rumble blips to perform
     */
    public void rumbleBlips(int count){
        g.rumbleBlips(count);
    }

    /**
     * Rumble the gamepad for a certain number of "blips" using set blip timing
     * This will displace any currently running rumble effect.
     * @param count the number of rumble blips to perform
     * @param power power of rumble from 0-1
     * @param duration the number of milliseconds per blip
     */
    public void rumbleBlips(int count, double power, int duration) {
        Gamepad.RumbleEffect.Builder builder = new Gamepad.RumbleEffect.Builder();

        for(int i = 0; i < count; i++) {
            builder.addStep(power,power,duration).addStep(0,0,100);
        }

        g.runRumbleEffect(builder.build());
    }

    /**
     * Rumble the gamepad for a certain number of "blips" using set blip timing
     * This will displace any currently running rumble effect.
     * @param count the number of rumble blips to perform
     * @param power power of rumble from 0-1
     * @param duration the number of milliseconds per blip
     * @param waitDuration the number of milliseconds between blips
     */
    public void rumbleBlips(int count, double power, int duration, int waitDuration) {
        Gamepad.RumbleEffect.Builder builder = new Gamepad.RumbleEffect.Builder();

        for(int i = 0; i < count; i++) {
            builder.addStep(power,power,duration).addStep(0,0,waitDuration);
        }

        g.runRumbleEffect(builder.build());
    }

    /**
     * Run an LED effect built using {@link Gamepad.LedEffect.Builder}
     * The LED effect will be run asynchronously; your OpMode will
     * not halt execution while the effect is running.
     *
     * Calling this will displace any currently running LED effect
     * @see Gamepad#runLedEffect(Gamepad.LedEffect ledEffect)
     */
    public void runLedEffect(Gamepad.LedEffect effect){
        g.runLedEffect(effect);
    }

    /**
     * Set the led of the gamepad with an RBG color for a certain duration
     * @param r red value of color
     * @param g green value of color
     * @param b blue value of color
     * @param duration milliseconds to set LED for, or {@link Gamepad#LED_DURATION_CONTINUOUS}
     * @see Gamepad#setLedColor(double r, double g, double b, int duration)
     */
    public void setLed(int r, int g, int b, int duration){
        this.g.setLedColor(r,g,b, duration);
    }

    /**
     * Set the led of the gamepad with an RBG color
     * @param r red value of color
     * @param g green value of color
     * @param b blue value of color
     * @see Gamepad#setLedColor(double r, double g, double b, int duration)
     */
    public void setLed(int r, int g, int b){
        this.g.setLedColor(r,g,b, Gamepad.LED_DURATION_CONTINUOUS);
    }

    /**
     * Returns all values of gpad
     * @return Returns all values of gpad
     */
    @NonNull
    @Override
    public String toString() {
        return String.format(
                Locale.ROOT,
                "X Button: %b\n" +
                        "Y Button (Or Triangle): %b\n" +
                        "A Button (Or Cross): %b\n" +
                        "B Button (Or Circle): %b\n" +
                        "Left Bumper: %b\n" +
                        "Right Bumper: %b\n" +
                        "Dpad Up: %b\n" +
                        "Dpad Left: %b\n" +
                        "Dpad Right: %b\n" +
                        "Dpad Down: %b\n" +
                        "Start Button: %b\n" +
                        "Back Button: %b\n" +
                        "Left Stick Button: %b\n" +
                        "Right Stick Button: %b\n" +
                        "Touchpad Clicked: %b\n" +
                        "Options Button: %b\n" +
                        "Share Button: %b\n" +
                        "PS Button: %b\n" +
                        "Guide Button: %b\n" +
                        "Square Button: %b\n" +
                        "Cross Button (Or X): %b\n" +
                        "Triangle Button (Or Y): %b\n" +
                        "Circle Button (Or B): %b\n" +
                        "Touchpad Top Left Click: %b\n" +
                        "Touchpad Top Right Click: %b\n" +
                        "Touchpad Bottom Left Click: %b\n" +
                        "Touchpad Bottom Right Click: %b\n" +
                        "Touchpad Top Left Tap: %b\n" +
                        "Touchpad Top Right Tap: %b\n" +
                        "Touchpad Bottom Left Tap: %b\n" +
                        "Touchpad Bottom Right Tap: %b\n" +
                        "Touchpad Finger 1 X Position: %.2f\n" +
                        "Touchpad Finger 1 Y Position: %.2f\n" +
                        "Touchpad Finger 2 X Position: %.2f\n" +
                        "Touchpad Finger 2 Y Position: %.2f\n" +
                        "Touchpad Finger 1 Present: %b\n" +
                        "Touchpad Finger 2 Present: %b\n" +
                        "Touchpad Finger 1 Pressed: %b\n" +
                        "Touchpad Finger 2 Pressed: %b\n" +
                        "Right Stick X Position: %.2f\n" +
                        "Right Stick Y Position: %.2f\n" +
                        "Left Stick X Position: %.2f\n" +
                        "Left Stick Y Position: %.2f\n" +
                        "Right Trigger Value: %.2f\n" +
                        "Left Trigger Value: %.2f",
                x, y, a, b, bL, bR, dU, dL, dR, dD,
                start, back, LStickButton, RStickButton, t,
                options, share, ps, guide, square, cross, triangle, circle,
                tTopLeft, tTopRight, tBotLeft, tBotRight,tTopLeftPress, tTopRightPress, tBotLeftPress, tBotRightPress,
                (g.touchpad_finger_1 ? t1x : 0), (g.touchpad_finger_1 ? t1y : 0), (g.touchpad_finger_2 ? t2x : 0), (g.touchpad_finger_2 ? t2y : 0), t1, t2, t1Press, t2Press,
                stickRX, stickRY, stickLX, stickLY, RTrigger, LTrigger
        );

    }

    /**
     * Returns the default {@link Gamepad#toString()} toString of the {@link Gamepad} class. It represents the raw values
     * @see Gamepad#toString()
     * @see Gamepad
     * @return The raw gamepad values
     */
    public String rawToString(){
        return g.toString();
    }
}
