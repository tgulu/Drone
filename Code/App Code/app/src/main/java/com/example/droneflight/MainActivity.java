package com.example.droneflight;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;

import java.net.DatagramSocket;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }
}

public class MainActivity extends AppCompatActivity {
    private SeekBar seekBar;
    private SeekBar      seekBar2;
    private TextView textView;
    private TextView     statustext;
    private TextView     dronedata;
    private ToggleButton ctlBtn;
    private ToggleButton cutoffBtn;
    private EditText editText1;
    private EditText     editText4;
    private EditText     editText5;

    private int     Throttle;
    private float   Roll;
    private float   Pitch;
    private int     Yaw=127;		// 0-255, middle - no yaw

    private float Kp, Ki, Kd;
    byte[]        BKp,BKd,BKi;

    private short rRoll;
    private short rPitch;
    private short rHDG;
    private short rALT;
    private byte  rYaw;
    private byte  rCtrl;
    private byte  rLoopTime;
    private short rThrottle;
    private short WifiAtt;
    private short BatLvl;

    private char[] anime={'-','\\','|','/'};
    private short animei=0;
    private short animeo=0;

    private static int BUFSIZE=25;
    private static int fq=50;
    byte[] sendbuf;
    byte[] receivebuf;
    DatagramSocket sockets;
    final byte[] bcastadr={(byte) 192,(byte) 168,(byte) 43,(byte) 255};

    private boolean dataready;
    private boolean threadsalive;
    private boolean threadralive;

}