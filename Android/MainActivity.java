package com.example.mdptesttest;

import android.annotation.SuppressLint;
import android.content.BroadcastReceiver;
import android.content.ClipData;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.graphics.PorterDuff;
import android.os.Bundle;
import android.os.Handler;
import android.text.TextUtils;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.app.AppCompatDelegate;
import androidx.appcompat.widget.SwitchCompat;
import androidx.core.content.ContextCompat;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity {

    Button button, btnSave, btnLoad;
    ImageButton btnClearMaze, btnStartAlgo;
    private TextView rbtStatus;
    private ImageView btIconView;
    private MapView mapView;
    private static BluetoothConnection bluetoothConnection;
    private StringBuilder receivedMsgText;
    public ObstacleView[] obstacleViews = new ObstacleView[8];
    private TextView textViewReceivedMessageContent;
    private String msgToSend ="";
    private Handler msgHandler = new Handler();
    SwitchCompat switchMode;
    boolean darkMode;
    public static ArrayList<String> savedString = new ArrayList<>();
    SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;
    Runnable repeatMessage = new Runnable() {
        @Override
        public void run() {
            sendMessage(msgToSend);
            msgHandler.postDelayed(this, 1500);
        }
    };

    private BroadcastReceiver msgReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String receivedMsg = intent.getStringExtra("receivedMsg");
            Log.d("MainActivity", "Loading Msg to received messaged");
            receiverHandler(receivedMsg);
        }
    };

    private BroadcastReceiver btStatusReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (action.equals("connectionStatus")) {
                boolean isConnected = intent.getBooleanExtra("isConnected", false);
                if (!isConnected) {
                    // Connection was lost, attempt to reconnect
                    updateBluetoothStatus();
                }
                else {
                    updateBluetoothStatus();
                }

            }
        }
    };

    @Override
    protected void onResume() {
        super.onResume();
        // Register the BroadcastReceiver to receive broadcasts with the "incomingMsg" action
        LocalBroadcastManager.getInstance(this).registerReceiver(msgReceiver, new IntentFilter("incomingMsg"));
        LocalBroadcastManager.getInstance(this).registerReceiver(btStatusReceiver, new IntentFilter("connectionStatus"));
    }

    @Override
    protected void onPause() {
        super.onPause();
        // Unregister the BroadcastReceiver to avoid memory leaks
        LocalBroadcastManager.getInstance(this).unregisterReceiver(msgReceiver);
        LocalBroadcastManager.getInstance(this).unregisterReceiver(btStatusReceiver);

    }

    private void setLongClickListenerItem(final ObstacleView item) {
        item.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View v) {
                ClipData data = ClipData.newPlainText("", "");
                View.DragShadowBuilder itemShadowBuilder = new View.DragShadowBuilder(v);
                v.startDragAndDrop(data,itemShadowBuilder,v,0);
                v.setVisibility(View.INVISIBLE);
                return true;
            }
        });
    }

    @SuppressLint("ClickableViewAccessibility")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mapView = findViewById(R.id.mapView);

        mapView.item1 = findViewById(R.id.itm1);
        mapView.item2 = findViewById(R.id.itm2);
        mapView.item3 = findViewById(R.id.itm3);
        mapView.item4 = findViewById(R.id.itm4);
        mapView.item5 = findViewById(R.id.itm5);
        mapView.item6 = findViewById(R.id.itm6);
        mapView.item7 = findViewById(R.id.itm7);
        mapView.item8 = findViewById(R.id.itm8);
        mapView.item2.setId("2");
        mapView.item3.setId("3");
        mapView.item4.setId("4");
        mapView.item5.setId("5");
        mapView.item6.setId("6");
        mapView.item7.setId("7");
        mapView.item8.setId("8");
        obstacleViews[0] = mapView.item1;
        obstacleViews[1] = mapView.item2;
        obstacleViews[2] = mapView.item3;
        obstacleViews[3] = mapView.item4;
        obstacleViews[4] = mapView.item5;
        obstacleViews[5] = mapView.item6;
        obstacleViews[6] = mapView.item7;
        obstacleViews[7] = mapView.item8;
        mapView.robot = findViewById(R.id.robot);

        btnClearMaze = findViewById(R.id.btnReset);
        btnClearMaze.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = getIntent();
                finish();
                startActivity(intent);
            }
        });

        btnStartAlgo = findViewById(R.id.btnStart);
        btnStartAlgo.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (BluetoothConnection.btConnectionStatus) {
                    sendMessage("START");
                }
            }
        });

        switchMode = findViewById(R.id.switchMode);
        sharedPreferences = getSharedPreferences("MODE", Context.MODE_PRIVATE);
        darkMode = sharedPreferences.getBoolean("darkMode", false);
        if (darkMode) {
            switchMode.setChecked(true);
            AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_YES);
            int colorGrid = ContextCompat.getColor(MainActivity.this, R.color.teal_700);
            int colorText = ContextCompat.getColor(MainActivity.this, R.color.white);
            btnClearMaze.setColorFilter(colorGrid, PorterDuff.Mode.SRC_IN);
            btnStartAlgo.setColorFilter(colorGrid,PorterDuff.Mode.SRC_IN);
            mapView.setGridPaint(colorGrid);
            for (int i = 0; i < obstacleViews.length; i++) {
                obstacleViews[i].setObsPaint(colorGrid, colorText);
            }
        }
        switchMode.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (darkMode) {
                    AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_NO);
                    editor = sharedPreferences.edit();
                    editor.putBoolean("darkMode", false);
                } else {
                        AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_YES);
                        editor = sharedPreferences.edit();
                        editor.putBoolean("darkMode", true);
                }
                    editor.apply();
            }
        });


        textViewReceivedMessageContent = findViewById(R.id.textViewReceivedMessageContent);

        setLongClickListenerItem(mapView.item1);
        setLongClickListenerItem(mapView.item2);
        setLongClickListenerItem(mapView.item3);
        setLongClickListenerItem(mapView.item4);
        setLongClickListenerItem(mapView.item5);
        setLongClickListenerItem(mapView.item6);
        setLongClickListenerItem(mapView.item7);
        setLongClickListenerItem(mapView.item8);
        mapView.setOnDragListener(mapView);

        rbtStatus = findViewById(R.id.robotStatus);
        btIconView = findViewById(R.id.btIcon);
        receivedMsgText = new StringBuilder();
        bluetoothConnection = new BluetoothConnection(MainActivity.this);

        updateBluetoothStatus();
        btnSave = (Button)findViewById(R.id.btnSave);
        btnSave.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                for (ObstacleView obs : obstacleViews) {
                    if (!obs.obstacleData.equals("Not Placed")) {
                        savedString.add(obs.obstacleData);
                    }
                }
                sharedPreferences = getSharedPreferences("MODE", Context.MODE_PRIVATE);
                String arrayAsString = TextUtils.join(";", savedString);
                editor = sharedPreferences.edit();
                editor.putString("stringArray", arrayAsString).apply();
                Log.d("ARRAYASSTRING", arrayAsString);
            }
        });

        btnLoad = (Button)findViewById(R.id.btnLoad);
        btnLoad.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sharedPreferences = getSharedPreferences("MODE", Context.MODE_PRIVATE);
                String arrayAsString = sharedPreferences.getString("stringArray", null);
                if (!arrayAsString.equals(null)){
                    Log.d("ARRAYASSTRING", arrayAsString);
                    mapView.clearAllMap();
                    String[] retrievedArray = arrayAsString.split(";");
                    for (String str : retrievedArray) {
                        Log.d("CHECK STRING", str);
                        String[] parts = str.substring("OBS,".length()).split(",");
                        if (parts.length == 4) {
                            int obs = Integer.parseInt(parts[0].trim());
                            int newY = Integer.parseInt(parts[1].trim());
                            int newX = Integer.parseInt(parts[2].trim());
                            String newDir = parts[3].trim();
                            mapView.setObsPos(obstacleViews[obs-1], newX, newY, newDir);
                            try {
                                Thread.sleep(200);
                            } catch (InterruptedException e) {
                                throw new RuntimeException(e);
                            }
                        }
                    }
                }
            }
        });
        button = (Button)findViewById(R.id.btn1);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent intent = new Intent(MainActivity.this, BluetoothScreen.class);
                startActivity(intent);
            }
        });
    }

    private void startSendingMessage(String message) {
        msgToSend = message;
        msgHandler.post(repeatMessage);
    }

    private void stopSendingMessage() {
        msgHandler.removeCallbacks(repeatMessage);
    }

    private void sendMessage(String message) {
        if (bluetoothConnection != null) {
            // Convert the message to bytes and send it via Bluetooth
            byte[] bytes = message.getBytes();
            BluetoothConnection.write(bytes);
        } else {
            Log.e("MainActivity", "BluetoothConnection is not initialized");
        }
    }

    public static void sendObstacle(String message) {
        if (BluetoothConnection.btConnectionStatus) {
            // Convert the message to bytes and send it via Bluetooth
            byte[] bytes = message.getBytes();
            BluetoothConnection.write(bytes);
        } else {
            Log.e("MainActivity", "BluetoothConnection is not initialized");
        }
    }

    public void receiverHandler(String text) {
        if (text.startsWith("ROBOT READY")) {
            rbtStatus.setText("READY TO START");
        }
        else if (text.startsWith("OBS")) {
            String[] parts = text.substring("OBS,".length()).split(",");
            if (parts.length == 2) {
                int obsNum = Integer.parseInt(parts[0].trim());
                String id = parts[1].trim();
                // setText to value for id
                obstacleViews[obsNum-1].setText(id);
                rbtStatus.setText("TARGET FOUND: OBS " + parts[0].trim() + ", ID: " + id);
            }
        }
        else if (text.startsWith("MOVING")) {
            rbtStatus.setText(text + "...");
        }
        else if (text.startsWith("CONNECTED TO RPI")) {
            rbtStatus.setText("CONNECTED TO RPI");
        }
        else if (text.startsWith("ROBOT END")) {
            rbtStatus.setText("TASK COMPLETED");
        }
        else if (text.startsWith("ROBOT,")) {
            //Split text to row, col, dir
            String[] parts = text.substring("ROBOT,".length()).split(",");
            if (parts.length == 3) {
                int newY = Integer.parseInt(parts[0].trim());
                int newX = Integer.parseInt(parts[1].trim());
                String newDir = parts[2].trim();
                mapView.robot.dir = newDir;
                mapView.setRobotPos(newY,newX,newDir);
            }
        }
        else {
            receivedMsgText.append(text + "\n");
            //textViewReceivedMessageContent.setText(receivedMsgText);
        }
    }

    private void updateBluetoothStatus() {
        Log.d("Main Activity", "STATUS UPDATE CHECK");
        if (BluetoothConnection.btConnectionStatus) {
            btIconView.setVisibility(View.VISIBLE);
            rbtStatus.setText("ROBOT CONNECTED");
        }
        else {
            btIconView.setVisibility(View.GONE);
            rbtStatus.setText("CONNECTING...");
        }
    }

}
