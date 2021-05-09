package com.example.dronecontrol;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.Arrays;

import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity implements MainBridge
{
    private SeekBar      seekBar;
    private SeekBar      seekBar2;
    private TextView     textView;
    private TextView     statustext;
    private TextView     dronedata;
    private ToggleButton ctlBtn;
    private ToggleButton cutoffBtn;
    private EditText     editText1;
    private EditText     editText4;
    private EditText     editText5;

    private int     Throttle;
    private float   Roll;
    private float   Pitch;
    private int     Yaw=127;		// 0-255, middle - no yaw

    private float Kp, Ki, Kd;
    byte[]        BKp,BKd,BKi;


    //animation showing udp being sent
    private char[] anime={'-','\\','|','/'};
    private short animei=0;
    private short animeo=0;

    private static int BUFSIZE=25;
    private static int fq=50;



    private boolean udpRecieveThreadAlive;



    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //setting screen buttons to xml variables
        //udpSendThreadAlive =false;
        udpRecieveThreadAlive =false;
        seekBar    = (SeekBar)  findViewById(R.id.seekBar1);
        seekBar2   = (SeekBar)  findViewById(R.id.seekBar2);
        textView   = (TextView) findViewById(R.id.TextView2);
        dronedata  = (TextView) findViewById(R.id.TextView1);
        statustext = (TextView) findViewById(R.id.textView3);
        ctlBtn     = (ToggleButton) findViewById(R.id.toggleButton1);
        cutoffBtn  = (ToggleButton) findViewById(R.id.toggleButton2);
        editText1  = (EditText) findViewById(R.id.editText1);
        editText4  = (EditText) findViewById(R.id.editText4);
        editText5  = (EditText) findViewById(R.id.editText5);

        BKp = new byte[Float.SIZE];
        BKi = new byte[Float.SIZE];
        BKd = new byte[Float.SIZE];
        Log.i("QCSV", "Activity creation");



        SensorManager sensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
        final float[] mValuesMagnet      = new float[3];
        final float[] mValuesAccel       = new float[3];
        final float[] mValuesOrientation = new float[3];
        final float[] mRotationMatrix    = new float[9];


        //

        final SensorEventListener mEventListener = new SensorEventListener()
        {
            public void onAccuracyChanged(Sensor sensor, int accuracy)
            {
                // Do something if sensor accuracy became low...
            }

            public void onSensorChanged(SensorEvent event)
            {
                switch (event.sensor.getType())
                {
                    case Sensor.TYPE_ACCELEROMETER:
                        System.arraycopy(event.values, 0, mValuesAccel, 0, 3);
                        break;
                    case Sensor.TYPE_MAGNETIC_FIELD:
                        System.arraycopy(event.values, 0, mValuesMagnet, 0, 3);
                        break;
                }
//                if ((mValuesAccel!= null) && (mValuesMagnet!=null))
//                {
                boolean success = SensorManager.getRotationMatrix(mRotationMatrix, null, mValuesAccel, mValuesMagnet);
                if (success)
                {
                    SensorManager.getOrientation(mRotationMatrix, mValuesOrientation);
                    Roll =(float) (mValuesOrientation[1]*180.0/3.14159);
                    Pitch=(float) (mValuesOrientation[2]*180.0/3.14159);
                    //newparams();
                }
//                }
            }
        };

        sensorManager.registerListener(mEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(mEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_NORMAL);




        DroneControl droneUI = new DroneControl(this);

        seekBar.setOnSeekBarChangeListener(new ControlBarChangeListener(Util.throttle, droneUI));

        seekBar2.setOnSeekBarChangeListener(new ControlBarChangeListener(Util.yaw, droneUI));


        final Handler trHandler = new Handler();

        //udpSendThreadAlive =true;


    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu)
    {
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item)
    {
        int id = item.getItemId();
        if (id == R.id.action_settings)
        {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    @Override
    protected void onDestroy()
    {
        //udpSendThreadAlive =false;
        udpRecieveThreadAlive =false;
        super.onDestroy();
    }

    //method to update pid vaules
    public void UpdatePIDValues (View v)
    {
        Kp=Float.valueOf(editText1.getText().toString());
        Kd=Float.valueOf(editText5.getText().toString());
        Ki=Float.valueOf(editText4.getText().toString());
        BKp=ByteBuffer.allocate(Float.SIZE).putFloat(Kp).array();
        BKd=ByteBuffer.allocate(Float.SIZE).putFloat(Kd).array();
        BKi=ByteBuffer.allocate(Float.SIZE).putFloat(Ki).array();
    }


    @Override
    public void updateAnimeo(char character) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                //this will run on UI thread, so its safe to modify UI views.
                textView.setText(String.valueOf(character));
            }
        });
    }

    @Override
    public void udpateReceiveAnimeo(char character) {

    }

    @Override
    public void updateDroneData(String data) {
        editText1.setText(data);
    }

    @Override
    public boolean getThreadAlive() {
        return udpRecieveThreadAlive;
    }


    @Override
    public boolean isControlButtonChecked() {
        return ctlBtn.isChecked();
    }

    @Override
    public boolean isCutoffButtonChecked() {
        return cutoffBtn.isChecked();
    }
}
