package com.example.dronecontrol;

import android.widget.SeekBar;

//seekbar update class
//scalable class that can add in pitch and roll in the future

public class ControlBarChangeListener implements SeekBar.OnSeekBarChangeListener {
    int controlType;

    DroneUI ui;
    public ControlBarChangeListener(int controlType, DroneUI ui) {

        this.controlType = controlType;
        this.ui = ui;
    }
    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
        switch (controlType){
            case Util.throttle:
                ui.updateThrottle(progress);
                break;
            case Util.yaw:
                ui.updateyaw(progress);
                break;
//             case Util.pitch:
//                 ui.updatePitch(progress);
//                 breakL;
        }

    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {

    }
}
