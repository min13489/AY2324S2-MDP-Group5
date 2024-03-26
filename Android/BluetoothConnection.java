package com.example.mdptesttest;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.util.Log;
import android.widget.Toast;

import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.Charset;
import java.util.UUID;
import static android.content.ContentValues.TAG;


public class BluetoothConnection {
    Context context;
    private BluetoothAdapter btAdapter;
    public static BluetoothDevice btDevice;
    private AcceptThread acceptThread;
    private ConnectThread connectThread;
    private static ConnectedThread connectedThread;
    public static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    public static boolean btConnectionStatus = false;
    private UUID deviceUUID;

    public BluetoothConnection(Context context) {
        this.btAdapter = BluetoothAdapter.getDefaultAdapter();
        this.context = context;
        start();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void checkPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            if (ContextCompat.checkSelfPermission(context,
                    android.Manifest.permission.ACCESS_BACKGROUND_LOCATION) != PackageManager.PERMISSION_GRANTED
                    && ContextCompat.checkSelfPermission(context,
                    android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            }
        }
    }

    private class AcceptThread extends Thread {
        private final BluetoothServerSocket mmServerSocket;

        public AcceptThread() {
            // Use a temporary object that is later assigned to mmServerSocket
            // because mmServerSocket is final.
            BluetoothServerSocket tmp = null;
            try {
                // MY_UUID is the app's UUID string, also used by the client code.
                checkPermission();
                tmp = btAdapter.listenUsingInsecureRfcommWithServiceRecord("GRP5", myUUID);
                Log.d("BluetoothConnection", "Setting up Server");
            } catch (IOException e) {
                Log.e(TAG, "Socket's listen() method failed", e);
            }
            mmServerSocket = tmp;
        }

        public void run() {
            BluetoothSocket socket = null;
            // Keep listening until exception occurs or a socket is returned.
            while (true) {
                try {
                    socket = mmServerSocket.accept();
                } catch (IOException e) {
                    Log.e(TAG, "Socket's accept() method failed", e);
                    break;
                }

                if (socket != null) {
                    // A connection was accepted. Perform work associated with
                    // the connection in a separate thread.
                    manageConnectedSocket(socket, btDevice);
                    try {
                        mmServerSocket.close();
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                    break;
                }
            }
        }

        // Closes the connect socket and causes the thread to finish.
        public void cancel() {
            try {
                mmServerSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Could not close the connect socket", e);
            }
        }
    }

    private class ConnectThread extends Thread {
        private BluetoothSocket mmSocket;

        public ConnectThread(BluetoothDevice device, UUID uuid) {
            btDevice = device;
            deviceUUID = uuid;
        }

        public void run() {
            BluetoothSocket tmp = null;
            Log.d("BluetoothConnection", "Connect thread started");
            try {
                Log.d("BluetoothConnection", "Trying to create Rfcomm socket using UUID");
                tmp = btDevice.createRfcommSocketToServiceRecord(deviceUUID);
            } catch (IOException e) {
                Log.e(TAG, "Could not close the create socket", e);
            }
            mmSocket = tmp;
            checkPermission();
            btAdapter.cancelDiscovery();

            try {
                checkPermission();
                mmSocket.connect();
                Log.d("BluetoothConnection", "Connect thread connected");

            } catch (IOException connectException) {
                try {
                    mmSocket.close();
                } catch (IOException closeException) {
                    Log.e(TAG, "Could not close the client socket", closeException);
                }
                Log.d("BluetoothConnection", "Could not connect to UUID");
            }
            manageConnectedSocket(mmSocket, btDevice);
        }

        // Closes the client socket and causes the thread to finish.
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Could not close the client socket", e);
            }
        }
    }
        public synchronized void start() {
            Log.d("BluetoothConnection", "Starting");
            if (connectThread != null) {
                connectThread.cancel();
                connectThread = null;
            }

            if (acceptThread == null) {
                acceptThread = new AcceptThread();
                acceptThread.start();
            }
        }

        public void startClient(BluetoothDevice device, UUID uuid) {
            Log.d("BluetoothConnection", "Client Starting");
            Toast.makeText(context, "Connecting Bluetooth, please wait...", Toast.LENGTH_LONG).show();
            connectThread = new ConnectThread(device, uuid);
            connectThread.start();
        }


    private class ConnectedThread extends Thread {
        private final BluetoothSocket socket;
        private final InputStream inputStream;
        private final OutputStream outputStream;

        public ConnectedThread(BluetoothSocket socket) {
            // Point in time when a connection has been made
            this.socket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;
            btConnectionStatus = true;
            Log.d(TAG, "ConnectedThread: CONNECTED TO BT DEVICE");
            Intent statusIntent = new Intent("connectionStatus");
            statusIntent.putExtra("isConnected", true);
            LocalBroadcastManager.getInstance(context).sendBroadcast(statusIntent);

            try {
                tmpIn = socket.getInputStream();
            } catch (IOException e) {
                Log.e(TAG, "Error occurred when creating input stream", e);
            }
            try {
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Log.e(TAG, "Error occurred when creating output stream", e);
            }

            inputStream = tmpIn;
            outputStream = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int numBytes;

            // Keep listening to the InputStream until an exception occurs.
            while (true) {
                try {
                    // Read from the InputStream.
                    numBytes = inputStream.read(buffer);
                    // Send the obtained bytes to the UI activity.
                    String msg = new String (buffer, 0, numBytes);
                    Log.d("BluetoothConnection", "Input Stream: " + msg);

                    Intent msgIntent = new Intent("incomingMsg");
                    msgIntent.putExtra("receivedMsg", msg);
                    LocalBroadcastManager.getInstance(context).sendBroadcast(msgIntent);

                } catch (IOException e) {
                    btConnectionStatus = false;
                    Log.d(TAG, "Input stream was disconnected", e);
                    Intent reconnectionIntent = new Intent("connectionStatus");
                    reconnectionIntent.putExtra("isConnected", false);
                    LocalBroadcastManager.getInstance(context).sendBroadcast(reconnectionIntent);
                    break;
                }
            }
        }

        // Call this from the main activity to send data to the remote device.
        public void write(byte[] bytes) {
            String text = new String(bytes, Charset.defaultCharset());
            Log.d(TAG, "Write: Writing to output stream: " + text);
            try {
                outputStream.write(bytes);
            } catch (IOException e) {
                Log.e(TAG, "Error occurred when sending data", e);
            }
        }

        // Call this method from the main activity to shut down the connection.
        public void cancel() {
            try {
                socket.close();
            } catch (IOException e) {
                Log.e(TAG, "Could not close the connect socket", e);
            }
        }

    }

    private void manageConnectedSocket(BluetoothSocket bluetoothSocket, BluetoothDevice bluetoothDevice) {
        Log.d("BluetoothConnection", "Connected starting");
        btDevice = bluetoothDevice;

        if(acceptThread != null){
            acceptThread.cancel();
            acceptThread = null;
        }

        connectedThread = new ConnectedThread(bluetoothSocket);
        connectedThread.start();
    }

    public static void write (byte[] out){
        Log.d("BluetoothConnection", "Write called");
        connectedThread.write(out);
    }

}

