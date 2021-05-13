package com.example.dronecontrol;

//class used for linking methods together

public interface MainBridge {


    void updateAnimeo(char character);

    void udpateReceiveAnimeo(char character);

    void updateDroneData(String data);

    boolean getThreadAlive();

    boolean isControlButtonChecked();

    boolean isCutoffButtonChecked();
}
