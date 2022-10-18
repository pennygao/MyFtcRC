package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SmartGamepad extends Gamepad implements PeriodicUpdateCallback {

    private Gamepad original = null;  // reference to the original gamepad object from opmode
    private Gamepad previous = null;  // a copy of gamepad with previous state

    public SmartGamepad(Gamepad gamepad) {
        original = gamepad;
        previous = new Gamepad();
        Utilities.getSharedUtility().registerForPeriodicUpdate(this);
        Log.v("gamepad", "Smart gamepad check in.");
    }

    @Override
    public void update () {
        try {
            previous.copy(this);
            copy(original);
        } catch (RobotCoreException e) {
            ;
        }
    }

    public boolean dpad_up_pressed() {
        return dpad_up && !previous.dpad_up;
    }

    public boolean dpad_down_pressed() {
        return dpad_down && !previous.dpad_down;
    }

    public boolean dpad_left_pressed() {
        return dpad_left && !previous.dpad_left;
    }

    public boolean dpad_right_pressed() {
        return dpad_right && !previous.dpad_right;
    }

    public boolean y_pressed() {
        return y && !previous.y;
    }

    public boolean dpad_changed() {
        Log.v("gamepad", String.format("Dpad (up/down/left/right): %b,%b,%b,%b, previous: %b,%b,%b,%b", dpad_up, dpad_down, dpad_left, dpad_right,
                previous.dpad_up, previous.dpad_down, previous.dpad_left, previous.dpad_right));

        boolean result =  (dpad_up ^ previous.dpad_up)     || (dpad_down ^ previous.dpad_down) ||
                (dpad_left ^ previous.dpad_left) || (dpad_right ^ previous.dpad_right);
        if (result) {
            Log.v("gamepad/transition", "dpad changed: True");
        }
        return result;
    }

    public boolean right_stick_released() {
        boolean result =  (Math.abs(right_stick_x) + Math.abs(right_stick_y) < 0.001) &&
                (Math.abs(previous.right_stick_x) + Math.abs(previous.right_stick_y) > 0.001);
        if (result) {
            Log.v("gamepad/transition", "right stick released: True");
        }
        return result;
    }
}