package org.firstinspires.ftc.teamcode.teamcode.generic_classes;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;

public class GamepadDriver {
    private Gamepad gamepad;
    public GamepadDriver(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    double touchpad_1_start_x = 0;
    double touchpad_1_start_y = 0;
    double touchpad_1_current_x = 0;
    double touchpad_1_current_y = 0;
    boolean touchpad_1_last_state = false;

    public List<Double> getTouchpadAsStick(){
        return null;
    }


    public void class_tick(){
        if((gamepad.touchpad_finger_1==true)&&(touchpad_1_last_state==false)){

        }
        touchpad_1_last_state = true;
    }
}
