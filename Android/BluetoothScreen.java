package com.example.mdptesttest;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class BluetoothScreen extends AppCompatActivity {

    Button backBtn, scanBtn;
    ListView pairedDeviceList, availDeviceList;
    private BluetoothConnection bluetoothConnection;
    private BluetoothAdapter btAdapter;
    private BluetoothDevice btDevice;
    private static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private ArrayList<String> pairedDeviceInfoList;
    private ArrayList<String> availDeviceInfoList;
    private ArrayAdapter<String> pairedArrayAdapter;
    private ArrayAdapter<String> availArrayAdapter;
    private IntentFilter scanIntentFilter = new IntentFilter(BluetoothAdapter.ACTION_SCAN_MODE_CHANGED);
    private BroadcastReceiver scanModeReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (action.equals(BluetoothAdapter.ACTION_SCAN_MODE_CHANGED)){
                int modeValue = intent.getIntExtra(BluetoothAdapter.EXTRA_SCAN_MODE, BluetoothAdapter.ERROR);
                if (modeValue == BluetoothAdapter.SCAN_MODE_CONNECTABLE){
                    Toast.makeText(getApplicationContext(), "Not in Discoverable mode but can still receive connections", Toast.LENGTH_SHORT).show();
                }
                else if (modeValue == BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE) {
                    Toast.makeText(getApplicationContext(), "In Discoverable Mode", Toast.LENGTH_SHORT).show();
                }
                else if (modeValue == BluetoothAdapter.SCAN_MODE_NONE) {
                    Toast.makeText(getApplicationContext(), "Cannot receive connections", Toast.LENGTH_SHORT).show();
                }
                else Toast.makeText(getApplicationContext(), "Error", Toast.LENGTH_SHORT).show();
            }
        }
    };

    private BroadcastReceiver pairedReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (action.equals(BluetoothDevice.ACTION_BOND_STATE_CHANGED)) {
                BluetoothDevice mbtDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                checkPermission();
                if (btDevice.getBondState() == BluetoothDevice.BOND_BONDED){
                    Log.d("BluetoothScreen", "Bond Bonded");
                    String deviceInfo = mbtDevice.getName() + "\n" + mbtDevice.getAddress();
                    pairedDeviceInfoList.add(deviceInfo);
                    availDeviceInfoList.remove(deviceInfo);
                    pairedArrayAdapter.notifyDataSetChanged();
                    availArrayAdapter.notifyDataSetChanged();
                    btDevice = mbtDevice;

                }
                else if (btDevice.getBondState() == BluetoothDevice.BOND_BONDING) {
                    Log.d("BluetoothScreen", "Bond Bonding");
                }
                else if (btDevice.getBondState() == BluetoothDevice.BOND_NONE) {
                    Log.d("BluetoothScreen", "Bond None");
                }
            }
        }
    };

    @Override
    protected void onDestroy() {
        super.onDestroy();

        try{
            unregisterReceiver(scanModeReceiver);
            unregisterReceiver(pairedReceiver);
        } catch (IllegalArgumentException exception){
            exception.printStackTrace();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void checkPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            if (ContextCompat.checkSelfPermission(BluetoothScreen.this,
                    android.Manifest.permission.ACCESS_BACKGROUND_LOCATION) != PackageManager.PERMISSION_GRANTED
            && ContextCompat.checkSelfPermission(BluetoothScreen.this,
                    android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(
                    BluetoothScreen.this,
                    new String[]{android.Manifest.permission.ACCESS_FINE_LOCATION, android.Manifest.permission.ACCESS_BACKGROUND_LOCATION},
                    11);
            }
        }
        if (!btAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        // Check if the requestCode matches the one used in requestPermissions
        if (requestCode == 11) {
            // Check if the ACCESS_BACKGROUND_LOCATION permission is granted
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // Permission granted, perform actions that require this permission
                // You may also want to check if Bluetooth is enabled here
                Log.d("Permission", "ACCESS_BACKGROUND_LOCATION granted");
            } else {
                // Permission denied, inform the user or take alternative actions
                Log.d("Permission", "ACCESS_BACKGROUND_LOCATION denied");
            }
        } else if (requestCode == 101) {
            // Check if Bluetooth permission is granted
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // Permission granted, perform actions that require this permission
                Log.d("Permission", "Bluetooth permission granted");
            } else {
                // Permission denied, inform the user or take alternative actions
                Log.d("Permission", "Bluetooth permission denied");
            }
        }
        // Add more conditions if you have multiple permissions to check
    }

    private ArrayList<String> getPairedDeviceList() {
        checkPermission();
        Set<BluetoothDevice> pairedDevices = btAdapter.getBondedDevices();
        ArrayList<String> deviceInfoList = new ArrayList<>();

        if (pairedDevices.size() > 0) {
            for (BluetoothDevice device : pairedDevices) {
                String deviceName = device.getName();
                String deviceHardwareAddress = device.getAddress(); // MAC address
                deviceInfoList.add(deviceName + "\n" + deviceHardwareAddress);
            }
        }
        return deviceInfoList;
    }

    public void startConnection() {
        startBTConnection(btDevice, myUUID);
    }
    public void startBTConnection(BluetoothDevice device, UUID uuid) {
        Log.d("BluetoothScreen", "Start BT connection");
        bluetoothConnection.startClient(device, uuid);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.d("BluetoothScreen", "Start BT page");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.bluetooth_screen);

        this.btAdapter = BluetoothAdapter.getDefaultAdapter();
        if (btAdapter == null) {
            Toast.makeText(BluetoothScreen.this, "This device does not have Bluetooth support", Toast.LENGTH_LONG).show();
            return;
        }

        pairedDeviceList = (ListView) findViewById(R.id.lstPairedDevice);
        availDeviceList = (ListView) findViewById(R.id.lstAvailDevice);
        backBtn = (Button) findViewById(R.id.btn1);
        scanBtn = (Button) findViewById(R.id.btn2);

        IntentFilter actionFoundFilter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        IntentFilter bondStateFilter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        registerReceiver(pairedReceiver, bondStateFilter);

        //get Paired device list
        pairedDeviceInfoList = getPairedDeviceList();
        pairedArrayAdapter = new ArrayAdapter<>(
                BluetoothScreen.this,
                android.R.layout.simple_list_item_1, pairedDeviceInfoList);
        pairedDeviceList.setAdapter(pairedArrayAdapter);

        //get avail Device List
        availDeviceInfoList = new ArrayList<>();
        availArrayAdapter = new ArrayAdapter<>(
            BluetoothScreen.this,
            android.R.layout.simple_list_item_1, availDeviceInfoList);

        BroadcastReceiver broadcastReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                String action = intent.getAction();
                Log.d("BluetoothScreen", "Received action: " + action);
                checkPermission();
                if (BluetoothDevice.ACTION_FOUND.equalsIgnoreCase(action)) {
                        BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                        String deviceName = device.getName();
                        String deviceHardwareAddress = device.getAddress(); // MAC address

                        if (deviceName != null) {
                            if (!(pairedDeviceInfoList.contains(deviceName + "\n" + deviceHardwareAddress))) {
                                availDeviceInfoList.add(deviceName + "\n" + deviceHardwareAddress);
                                Log.d("BluetoothScreen", "Device found: " + deviceName + " (" + deviceHardwareAddress + ")");
                            }
                        }
                        /*
                        else {
                            availDeviceInfoList.add("Unknown Device" + "\n" + deviceHardwareAddress);
                            Log.d("BluetoothScreen", "Device found: " + "Unknown Device" + " (" + deviceHardwareAddress + ")");

                        }
                        */
                    availArrayAdapter.notifyDataSetChanged();
                }
            }

        };

        btAdapter.startDiscovery();
        registerReceiver(broadcastReceiver, actionFoundFilter);
        availDeviceList.setAdapter(availArrayAdapter);

    backBtn.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            Intent intent = new Intent(BluetoothScreen.this, MainActivity.class);
            startActivity(intent);
        }
    });

    scanBtn.setOnClickListener(new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
            discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 120);
            checkPermission();
            startActivity(discoverableIntent);
        }
    });

    registerReceiver(scanModeReceiver, scanIntentFilter);

    pairedDeviceList.setOnItemClickListener(new AdapterView.OnItemClickListener() {
        @Override
        public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
            String deviceInfo = (String) adapterView.getItemAtPosition(i);
            String[] deviceInfoParts = deviceInfo.split("\n");
            String deviceName = deviceInfoParts[0];
            String deviceAddress = deviceInfoParts[1];
            Log.d("BluetoothScreen", "Device pairing: " + deviceName + " (" + deviceAddress + ")");
            BluetoothDevice selectedDevice = btAdapter.getRemoteDevice(deviceAddress);
            btDevice = selectedDevice;
            bluetoothConnection = new BluetoothConnection(BluetoothScreen.this);
            startConnection();
            BluetoothConnection.btDevice = selectedDevice;
        }
    });

    availDeviceList.setOnItemClickListener(new AdapterView.OnItemClickListener() {
        @Override
        public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
            checkPermission();
            btAdapter.cancelDiscovery();

            String deviceInfo = (String) adapterView.getItemAtPosition(i);
            String[] deviceInfoParts = deviceInfo.split("\n");
            String deviceName = deviceInfoParts[0];
            String deviceAddress = deviceInfoParts[1];
            Log.d("BluetoothScreen", "Device pairing: " + deviceName + " (" + deviceAddress + ")");
            BluetoothDevice selectedDevice = btAdapter.getRemoteDevice(deviceAddress);
            selectedDevice.createBond();
            btDevice = selectedDevice;
        }
    });
}

}
