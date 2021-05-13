package com.example.dronecontrol;

import java.nio.ByteBuffer;


public class DroneControl implements DroneUI {


    public short rHDG;
    public short rALT;
    public short BatLvl;
    public short WifiAtt;
    public byte rLoopTime;
    private int     Throttle;
    private float   Roll;
    private float   Pitch;
    private int     Yaw;

    public short rRoll;
    public short rPitch;
    public byte  rYaw;
    public byte  rCtrl;
    public short rThrottle;



    private float Kp = 2;
    private float Ki= 0.8f;
    private float Kd = 0 ;
    byte[]        BKp,BKd,BKi = new byte[Float.SIZE];

    MainBridge main;

    UDP udp;

    //setting up the sending thread to take the udp data
    public DroneControl(MainBridge main){

        this.main = main;
        udp = new UDP();
        initSendBuf(udp);
        UDP.UDPSend udpSend = new UDP.UDPSend(main, udp);

        Thread UDPsendThread = new Thread(udpSend);
        UDPsendThread.setPriority((Thread.MAX_PRIORITY + Thread.NORM_PRIORITY)/2);
        UDPsendThread.start();


        UDP.UDPReceive udpReceive = new UDP.UDPReceive(main, udp, this);

        Thread udpRevthread = new Thread(udpReceive);


        udpRevthread.start();

    }


    //data that's being sent in the UDP packets to the phone
    void initSendBuf(UDP udp){

        BKp = ByteBuffer.allocate(Float.SIZE).putFloat(Kp).array();
        BKd = ByteBuffer.allocate(Float.SIZE).putFloat(Kd).array();
        BKi = ByteBuffer.allocate(Float.SIZE).putFloat(Ki).array();
        //udp packets sent to drone
        udp.sendbuf[0] ='D';		// ->drone protocol ID
        udp.sendbuf[1] ='R';
        udp.sendbuf[2] ='O';
        udp.sendbuf[3] ='N';
        udp.sendbuf[4] ='E';
        udp.sendbuf[5] =(byte)Throttle;
        udp.sendbuf[6] =(byte)Math.round(Roll+90);
        udp.sendbuf[7] =(byte)Math.round(Pitch+90);
        udp.sendbuf[8] =(byte)Yaw;
        udp.sendbuf[9] =(byte)((main.isControlButtonChecked())?1:0);		//checks to see of the power off button has been switched
        udp.sendbuf[10]=BKp[3];
        udp.sendbuf[11]=BKp[2];
        udp.sendbuf[12]=BKp[1];
        udp.sendbuf[13]=BKp[0];
        udp.sendbuf[14]=BKd[3];
        udp.sendbuf[15]=BKd[2];
        udp.sendbuf[16]=BKd[1];
        udp.sendbuf[17]=BKd[0];
        udp.sendbuf[18]=BKi[3];
        udp.sendbuf[19]=BKi[2];
        udp.sendbuf[20]=BKi[1];
        udp.sendbuf[21]=BKi[0];
        udp.sendbuf[22]=(byte)((main.isCutoffButtonChecked())?1:0);
        udp.sendbuf[23]=0;
        udp.dataready = true;
    }




    //method for taking updated throttle vaules
    @Override
    public void updateThrottle(int throttle) {
        Throttle = throttle;
        initSendBuf(udp);
    }

    //method for taking updated yaw vaules
    @Override
    public void updateyaw(int yaw) {
        Yaw = yaw;
        initSendBuf(udp);

    }






}
