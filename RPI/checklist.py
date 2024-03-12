#!/usr/bin/env python3

import bluetooth
import serial

import picamera
import base64
import requests

import os, sys
from time import sleep

# switches
taskA1 = False
taskA2 = False
taskA3 = True
taskA4 = False
taskA5 = False

# establish bluetooth connection with android
'''
os.system("sudo hciconfig hci0 piscan") # make discoverable
server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
#server_sock.bind(("", bluetooth.PORT_ANY))
server_sock.bind(("", 1))
server_sock.listen(1)

port = server_sock.getsockname()[1]

uuid = "00001101-0000-1000-8000-00805F9B34FB"
'''
'''
bluetooth.advertise_service(server_sock, "SampleServer", service_id=uuid,
                            service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                            profiles=[bluetooth.SERIAL_PORT_PROFILE],
                            # protocols=[bluetooth.OBEX_UUID]
                            )
'''
'''
print("Waiting for bt connection on RFCOMM channel", port)

client_sock, client_info = server_sock.accept()
print("Accepted connection from", client_info)
'''
# establish serial connection with stm
ttyUSB = [filename for filename in os.listdir("/dev") if filename.startswith("ttyUSB")]
if len(ttyUSB) == 0:
    print("Serial not connected")
    sys.exit(0)

serial_link = serial.Serial("/dev/{}".format(ttyUSB[0]), 115200)
print("Serial connected")
#serial_link.write("wx021".encode(encoding="ascii"))
#print("wx021 sent")
#sleep(5)
# send commands
if taskA1:
    print("Running task A1")
    try:
        while True:
            data = client_sock.recv(1024)
            if not data:
                break
            print("Received", data)
            if data == b'FW':
                serial_link.write('wx010'.encode(encoding="ascii"))
    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        print("Ctrl+C pressed")
        
elif taskA2:
    print("Running task A2")
    api_ip = "192.168.5.29"
    obsNo = 1
    with picamera.PiCamera() as camera:
        camera.resolution = (640,640)
        camera.start_preview()
        sleep(2)
        camera.capture(f"OBS{obsNo}.jpg")
        camera.close()

    url = "http://{}:5000/test-image".format(api_ip)
    with open(f"OBS{obsNo}.jpg", 'rb') as f:
        image = f.read()
    payload = {
        "image":base64.b64encode(image).decode('utf-8'),
        "image_type": "jpg",
        "obs": 99
    }

    headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
    response = requests.post(url, headers=headers, json=payload)
    try:
        data = response.json()
        print(data)
    except:
        print("No response")
        
elif taskA3:
    print("Running task A3")
    if len(sys.argv) == 2:
        serial_link.write("{}".format(sys.argv[1].strip()).encode(encoding="ascii"))
    else:
        command = input("Enter the command [5 char] => ")
        serial_link.write("{}".format(command.strip()).encode(encoding="ascii"))
elif taskA4:
    print("Running task A4")
    for i in range(3):
        serial_link.write("&&081".encode(encoding="ascii"))
        print("send")
        sleep(8)

elif taskA5:
    print("Running task A5")
    imageNotFound = True
    while imageNotFound:
        # back > turn right > forward (opt) > turn left > back (opt) > turn left > forward (opt)
        commands = ["sx030","wd090","wx010","wa090","sx030","wa090","wx010"]
        for command in commands:
            serial_link.write(command.encode(encoding="ascii"))
            sleep(5) # does the robot listen for command while moving?
        
        # take pic
        api_ip = "192.168.5.29"
        obsNo = 1
        with picamera.PiCamera() as camera:
            camera.resolution = (640,640)
            camera.start_preview()
            sleep(2)
            camera.capture(f"OBS{obsNo}.jpg")
            camera.close()

        url = "http://{}:5000/test-image".format(api_ip)
        with open(f"OBS{obsNo}.jpg", 'rb') as f:
            image = f.read()
        payload = {
            "image":base64.b64encode(image).decode('utf-8'),
            "image_type": "jpg",
            "obs": 99
        }

        headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
        response = requests.post(url, headers=headers, json=payload)
        try:
            data = response.json()
            print(data)
            
            if data["id"] != 99:
                imageNotFound = False # aka found, stop the loop
        except:
            print("No response")

print("Disconnected.")

#client_sock.close()
#server_sock.close()
print("All done.")
