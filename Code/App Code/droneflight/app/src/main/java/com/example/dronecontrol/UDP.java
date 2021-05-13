package com.example.dronecontrol;

import android.util.Log;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.Arrays;


//class deals with the UDP communications between the app and the drone
public class UDP {

    private static int BUFSIZE = 25;
    private static int fq = 50;
    byte[] sendbuf = new byte[BUFSIZE];
    byte[] receivebuf = new byte[BUFSIZE];
    boolean dataready = true;


    private char[] anime = {'-', '\\', '|', '/'};
    int animeIndex = 0;

    //ip address of the drone once connected
    DatagramSocket sockets;
    final byte[] bcastadr = {(byte) 192, (byte) 168, (byte) 0, (byte) 38};
    //192.168.0.38

    public UDP(){
        try
        {
            sockets = new DatagramSocket();
            sockets.setBroadcast(true);
        }
        catch (SocketException e)
        {
            e.printStackTrace();
            Log.e("DRON",e.getMessage());
        }
    }

    //sending the data to the drone
    static class UDPSend implements Runnable {
        MainBridge main;

        UDP udpSettings;

        public UDPSend(MainBridge main, UDP udp) {
            this.main = main;
            this.udpSettings = udp;
        }

        @Override
        public void run() {
            // sending UDP packets to the drone
            while (true) {
                if (udpSettings.dataready) {

                    Log.d("UDP", "sending packet");
                    try {
                        // sending data to ip address, on the port
                        DatagramPacket packet = new DatagramPacket(udpSettings.sendbuf, BUFSIZE, InetAddress.getByAddress(udpSettings.bcastadr), 5800);
                        udpSettings.sockets.send(packet);

                        udpSettings.animeIndex++;
                        if (udpSettings.animeIndex == 4) udpSettings.animeIndex = 0;
                        main.updateAnimeo(udpSettings.anime[udpSettings.animeIndex]);


                    } catch (SocketException e) {
//						  statustext.setText(e.getMessage());
                        Log.e("DRON", e.getMessage());
                        e.printStackTrace();
                    } catch (IOException e) {
//						statustext.setText(e.getMessage());
                        Log.e("DRON", e.getMessage());
                        e.printStackTrace();
                    }
                    //udpSettings.dataready = false;

                    try {
                        Thread.sleep(fq);
                    } catch (InterruptedException e) {
//						statustext.setText(e.getMessage());
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    //receiving the data from the drone
    static class UDPReceive implements Runnable{

        MainBridge main;
        UDP udpSettings;
        DroneControl drone;

        public UDPReceive(MainBridge main, UDP udp, DroneControl drone) {
            this.main = main;
            this.udpSettings = udp;
            this.drone = drone;

        }

        @Override
        public void run() {
            while (true)
            {
                try
                {
                    DatagramPacket packet = new DatagramPacket(udpSettings.receivebuf, BUFSIZE);
                    Arrays.fill(udpSettings.receivebuf,(byte)0);
                    udpSettings.sockets.receive(packet);
                    if ((udpSettings.receivebuf[0]=='D') && (udpSettings.receivebuf[1]=='R') && (udpSettings.receivebuf[2]=='O') && (udpSettings.receivebuf[3]=='N') && (udpSettings.receivebuf[4]=='E'))
                    {
                        drone.rRoll     = (short) (((udpSettings.receivebuf[5] << 8) & 0x0000ff00) | (udpSettings.receivebuf[6] & 0x000000ff));
                        drone.rPitch    = (short) (((udpSettings.receivebuf[7] << 8) & 0x0000ff00) | (udpSettings.receivebuf[8] & 0x000000ff));
                        drone.rHDG      = (short) (((udpSettings.receivebuf[9] << 8) & 0x0000ff00) | (udpSettings.receivebuf[10] & 0x000000ff));
                        drone.rALT      = (short) (((udpSettings.receivebuf[11]<< 8) & 0x0000ff00) | (udpSettings.receivebuf[12] & 0x000000ff));
                        drone.BatLvl    = (short) (((udpSettings.receivebuf[15]<< 8) & 0x0000ff00) | (udpSettings.receivebuf[16] & 0x000000ff));
                        drone.rThrottle = (short) (udpSettings.receivebuf[13] & 0x000000ff);
                        drone.WifiAtt   = (short) (udpSettings.receivebuf[14] & 0x000000ff);
                        drone.rYaw      =  (byte) (udpSettings.receivebuf[17] & 0x000000ff);
                        drone.rCtrl     =  (byte) (udpSettings.receivebuf[18] & 0x000000ff);
                        drone.rLoopTime =  (byte) (udpSettings.receivebuf[19] & 0x000000ff);

                        String s="  DRONE:\nBank=";
                        s+=Integer.toString(drone.rRoll-90);
                        s+="\nPitch=";
                        s+=Integer.toString(drone.rPitch-90);
                        s+=Short.toString((short)(drone.rThrottle*100/255));
                        s+="%";
                        s+=Short.toString(drone.BatLvl);
                        s+="mV";
                        if (drone.rCtrl==0) s+="OFF";
                        if (drone.rCtrl==1) s+="ON";
                        s+="\nLoop(ms)=";
                        s+=Integer.toString(drone.rLoopTime);
                        s+="\n";

                        udpSettings.animeIndex++;
                        if (udpSettings.animeIndex==4) udpSettings.animeIndex=0;
                        main.udpateReceiveAnimeo(udpSettings.anime[udpSettings.animeIndex]);

                        main.updateDroneData(s);
                    }
                }
                catch (SocketException e)
                {
//				    statustext.setText(e.getMessage());
                    e.printStackTrace();
                }
                catch (IOException e)
                {
//					statustext.setText(e.getMessage());
                    e.printStackTrace();
                }
                catch (Exception e)
                {
//					statustext.setText(e.getMessage());
                    e.printStackTrace();
                }
            }

        }
    }

}



