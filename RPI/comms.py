### IMPORTS ###

# general
from time import sleep
import os, subprocess, logging, sys
# connections related
import bluetooth, socket, serial, requests
# multithreading related
from multiprocessing import Process, Manager
from ctypes import c_bool
import queue
# processing related
import base64, picamera

### SWITCHES ###

obstacleCourse = 1           # 1 - obstacle course 0 - fastest car
# logLevel = logging.INFO      # logging.INFO - normal run
logLevel = logging.DEBUG     # logging.DEBUG - for debug msgs

### GLOBALS ###

robotPos = [2,2,'N']            # occupying bottom left corner (9 squares)
api_ip = "192.168.5.29"         # min's computer IP
# api_ip = "192.168.5.22"         # yen's computer IP
# api_ip = "192.168.5.23"         # dext's computer IP

### CODES ###

# SETUP - android connection class
class AndroidBT:
    def __init__(self):
        self.client_sock = None
        self.server_sock = None
    
    def connect(self):
        logging.debug("establishing bluetooth connection")
        try:
            # make discoverable
            os.system("sudo hciconfig hci0 piscan")

            # make sure channel 1 available
            command = "sudo ps -aux | grep 'rfcomm listen /dev/rfcomm0 1'"
            p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            data, err = p.communicate()
            data = data.decode("utf-8")
            lines = data.split("\n")
            for line in lines:
                if line.startswith("root"):
                    pid = line.split("root")[1].strip().split()[0].strip()
                    subprocess.run(["sudo", "kill", "-9", pid], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    break
            
            # start server socket
            self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.server_sock.bind(("",bluetooth.PORT_ANY))
            self.server_sock.listen(1)

            # important info
            port = self.server_sock.getsockname()[1]
            uuid = "00001101-0000-1000-8000-00805F9B34FB"
            logging.info("waiting for bluetooth connection on RFCOMM channel {}".format(port))

            # receiving incoming connection
            self.client_sock, client_info = self.server_sock.accept()
            logging.info("accepted conenction from {}".format(client_info))
            return True
        except:
            logging.error("bluetooth connection unsuccessful")
            self.server_sock.close()
            self.client_sock.close()
            self.server_sock = None
            self.client_sock = None
            return False
    
    def disconnect(self):
        if self.server_sock is None:
            return
        try:
            self.server_sock.shutdown(socket.SHUT_RDWR)
            self.client_sock.shutdown(socket.SHUT_RDWR)
            self.server_sock.close()
            self.client_sock.close()
            self.server_sock = None
            self.client_sock = None
            logging.info("bluetooth disconnected")
        except:
            logging.error("bluetooth disconnection unsuccessful")

    def send(self, msg):
        try:
            self.client_sock.send(msg.encode("utf-8"))
            logging.debug("send android msg: {}".format(msg))
        except:
            logging.error("error sending to android: msg - {}".format(msg))

    def receive(self):
        try:
            msg = self.client_sock.recv(1024)
            if msg is not None or msg.strip() != "":
                logging.debug("recv android msg: {}".format(msg.strip().decode("utf-8")))
                return msg.strip().decode("utf-8")
        except:
            logging.error("error receiving from android")

# SETUP - stm connection class
class STMSerial:
    def __init__(self):
        self.serial_link = None

    def connect(self):
        # get the ttyUSB in use
        ttyUSB = [filename for filename in os.listdir("/dev") if filename.startswith("ttyUSB")]
        if len(ttyUSB) == 0:
            logging.error("serial not connected")
            return False
        
        # connect
        self.serial_link = serial.Serial("/dev/{}".format(ttyUSB[0]), 115200)
        logging.info("serial connected")
        return True

    def disconnect(self):
        if self.serial_link is None:
            return
        self.serial_link.close()
        self.serial_link = None
        logging.info("serial disconnected")

    def send(self, msg):
        self.serial_link.write(msg.strip().encode(encoding="ascii"))
        logging.debug("stm msg sent: {}".format(msg))

    def receive(self):
        try:
            msg = self.serial_link.read(3)
            if msg is not None:
                # logging.debug("stm msg recv: {}".format(msg.strip().decode(encoding="ascii")))
                return msg.strip().decode(encoding="ascii")
        except:
            # logging.error("error receiving from stm")
            pass
        
# SETUP - main controller class
class Brain:
    # INIT - set up connection objects and multiprocessing pre-reqs
    def __init__(self):
        # setup for both tasks
        self.STM = STMSerial()                      # object - STM conn
        self.manager = Manager()                    # manager - multiprocessing
        self.movement_lock = self.manager.Lock()    # lock - per command
        self.stm_sendq = self.manager.Queue()       # queue - STM commands
        self.rpi_queue = self.manager.Queue()       # queue - RPI tasks
        self.proc_sendSTM = None                    # process - sending messages from STM
        self.proc_recvSTM = None                    # process - receiving messages from STM
        self.proc_rpi = None                        # process - do RPI things
        self.android = AndroidBT()                          # object - Android conn
        self.android_dropped = self.manager.Event()         # event - Android disconnected
        self.android_sendq = self.manager.Queue()           # queue - Android display messages
        self.proc_sendAndroid = None                        # process - sending messages to Android
        self.proc_recvAndroid = None                        # process - receiving messages from Android 
        self.proc_robotRun = None                           # process - robot run commands

        # only for task 1
        if obstacleCourse:
            self.path_obtained = self.manager.Event()           # event - path obtained or not
            self.send_pic = self.manager.Event()                # event - image sent
            self.obs_queue = self.manager.Queue()               # queue - obs list
            self.state_queue = self.manager.Queue()             # queue - states list
            self.commandq = self.manager.Queue()                # queue - commands
            self.inserting = self.manager.Event()               # event - command insertion
            self.duplicating = self.manager.Event()             # event - state duplication
        else:
            self.arrow_recog = self.manager.Event()             # event - arrow recognised

    # FUNCTION - check if API is alive using curl
    def pingAPI(self):
        output = subprocess.run(["curl", "http://{}:5000/".format(api_ip)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL).stdout.strip().decode("utf-8")
        if output == "API is up and running":
            logging.info("API up")
            return True
        else:
            logging.error("API down, check if connected to subnet")
            return False

    # FUNCTION - main process
    def run(self):
        try:
            # establish connections
            conn1, conn2, conn3 = False, False, False
            conn1 = self.android.connect()
            self.android_sendq.put("CONNECTED TO RPI")
            conn2 = self.STM.connect()
            conn3 = self.pingAPI()
            
            if not (conn1 and conn2 and conn3):
                self.android.disconnect()
                self.STM.disconnect()
                logging.error("some connection not properly established. aborted")
                self.stop()  # exit program if connection not ready
            
            # start processes
            self.proc_sendAndroid = Process(target=self.sendAndroid)
            self.proc_sendAndroid.start()
            self.proc_recvAndroid = Process(target=self.recvAndroid)
            self.proc_recvAndroid.start()
            self.proc_sendSTM = Process(target=self.sendSTM)
            self.proc_sendSTM.start()
            self.proc_recvSTM = Process(target=self.recvSTM)
            self.proc_recvSTM.start()
            self.proc_rpi = Process(target=self.rpiTasks)
            self.proc_rpi.start()
            self.proc_robotRun = Process(target=self.robotRun)
            self.proc_robotRun.start()
            logging.info("processes started")

            # initial messages
            self.android_sendq.put("ROBOT READY")   # enable choosing of obstacles
            # self.STM.warmup()
            self.reconnect_android()                # waiting and test for disconnection

        except KeyboardInterrupt:
            self.stop()
    
    # FUNCTION - stop this code
    def stop(self):
        self.android.disconnect()
        try:
            self.proc_sendAndroid.kill()
            self.proc_recvAndroid.kill()
        except:
            pass
        self.STM.disconnect()
        
        sys.exit(0)
        logging.info("program exit")

    # FUNCTION - monitor for android connection dropped
    def reconnect_android(self):
        logging.info("watching for android disconnection")

        while True:
            # wait for connection drop
            self.android_dropped.wait()
            logging.error("android down")

            # cleanup
            self.proc_sendAndroid.kill()
            self.proc_recvAndroid.kill()
            self.proc_sendAndroid.join()
            self.proc_recvAndroid.join()
            assert self.proc_sendAndroid.is_alive() is False
            assert self.proc_recvAndroid.is_alive() is False
            self.android.disconnect()
            
            # reconnection
            self.android.connect()
            self.proc_sendAndroid = Process(target=self.sendAndroid)
            self.proc_sendAndroid.start()
            self.proc_recvAndroid = Process(target=self.recvAndroid)
            self.proc_recvAndroid.start()

            # information and clear flag
            logging.info("android reconnected")
            self.android_sendq("RECONNECTED")
            self.android_dropped.clear()
    
    # FUNCTION - send messages to Android (child process)
    def sendAndroid(self):
        while True:
            try:
               msg = self.android_sendq.get(timeout=0.5)    # get queue item every .5s
            except queue.Empty:
                continue

            try:
                self.android.send(msg)
                if msg.startswith("OBS"):
                    self.send_pic.set()
            except OSError:
                self.android_dropped.set()

    # FUNCTION - receive messages from Android (child process)     
    def recvAndroid(self):
        while True:
            # retrieve message
            msg_str = None
            try:
                msg_str = self.android.receive()
            except OSError:
                self.android_dropped.set()
            
            if msg_str is None:
                continue
            
            # interpret message
            ### msg format:
            # (A) OBS,obstacle_id,row,col,dir       - information about one obstacle
            # (B) CLEAR,obstacle_id                 - remove one obstacle
            # (C) START                             - start robot on path
            msg_parts = msg_str.split(",")
            # case A - insertion
            if msg_parts[0] == "OBS":
                obs_item = [20-int(msg_parts[2]), int(msg_parts[3])+1, msg_parts[4], int(msg_parts[1])]
                self.obs_queue.put(obs_item)
                logging.info("obstacle inserted - {}".format(str(obs_item)))
            # case B - removal
            elif msg_parts[0] == "CLEAR":
                toKeep = []
                while not self.obs_queue.empty():
                    curObs = self.obs_queue.get()
                    if curObs[3] != int(msg_parts[1]):
                        toKeep.append(curObs)
                    else:
                        logging.info("obstacle removed - {}".format(str(curObs)))
                    
                for obs in toKeep:
                    self.obs_queue.put(obs)
            # case C - start
            elif msg_parts[0] == "START":
                if not self.pingAPI():
                    logging.error("API down cannot start path")
                    self.android_sendq.put("API down, cannot start")
                if self.obs_queue.empty() and obstacleCourse:
                    logging.error("no obstacles cannot start path")
                    self.android_sendq.put("No obstacles")
                else:
                    if obstacleCourse:
                        self.rpi_queue.put("PATH")  # ask RPI to get path
                        self.path_obtained.wait()   # wait for RPI to obtain path from algo
                        self.android_sendq.put("MOVING")
                    else:
                        self.commandq.put("T2START")
        
    # FUNCTION - send commands to STM (child process)
    def sendSTM(self):
        while True:
            try:
                try:
                    msg = self.stm_sendq.get(timeout=0.5)
                except queue.Empty:
                    continue
                self.STM.send(msg)
            except KeyboardInterrupt:
                break
    
    # FUNCTION - receive ack from STM (child process)
    def recvSTM(self):
        while True:
            msg = self.STM.receive()
            if msg is None:
                continue
            logging.debug("msg: {}".format(msg))
            if msg.startswith("A") or msg.startswith("C") or msg.startswith("K"):
                try:
                    currentPos = self.state_queue.get()
                    if currentPos != "KEEP":
                        self.android_sendq.put("ROBOT,{},{},{}".format(str(20-currentPos[0]),str(currentPos[1]-1),currentPos[2]))
                    # logging.debug(str(currentPos))
                    sleep(2)
                    self.movement_lock.release()
                except Exception as e:
                    print(e)
                    logging.error("tried to release a released lock")

    def robotRun(self):
        prevFL = False
        prevFR = False
        prevBL = False
        prevBR = False
        while True:
            try:
                # get command
                try:
                    command = self.commandq.get(timeout=0.5)    # get queue item every .5s
                except queue.Empty:
                    continue
                
                # get permissions
                self.movement_lock.acquire()
                
                # DEBUG
                logging.debug("note issue and move to by right location")
                sleep(3)
                # command reading
                ## FW and BW movements
                logging.debug(command)
                
                if command.startswith("FW"):
                    distance = int(command[2:])
                    send_str = "wx{:03d}".format(distance*10)
                    self.stm_sendq.put(send_str)
                elif command.startswith("wx"):
                    self.stm_sendq.put(command)
                elif command.startswith("BW"):
                    distance = int(command[2:])
                    send_str = "sx{:03d}".format(distance*10)
                    self.stm_sendq.put(send_str)
                elif command.startswith("sx"):
                    self.stm_sendq.put(command)

                ## TURN movements
                elif command.startswith("FL"):
                    if prevFL:
                        self.stm_sendq.put("fa090")
                        prevFL = False
                    else:
                        prevFL = True
                        self.insertCommand(["wx005","FL00","wx003"])
                        self.inserting.wait()
                        self.inserting.clear()
                        self.dupStates(1,2)
                        self.duplicating.wait()
                        self.duplicating.clear()
                        self.movement_lock.release()
                elif command.startswith("FR"):
                    if prevFR:
                        self.stm_sendq.put("fd090")
                        prevFR = False
                    else:
                        prevFR = True
                        self.insertCommand(["wx006","FR00","sx006"])
                        self.inserting.wait()
                        self.inserting.clear()
                        self.dupStates(1,2)
                        self.duplicating.wait()
                        self.duplicating.clear()
                        self.movement_lock.release()
                elif command.startswith("BL"):
                    if prevBL:
                        self.stm_sendq.put("ba090")
                        prevBL = False
                    else:
                        prevBL = True
                        self.insertCommand(["sx003","BL00"])
                        self.inserting.wait()
                        self.inserting.clear()
                        self.dupStates(1,1)
                        self.duplicating.wait()
                        self.duplicating.clear()
                        self.movement_lock.release()
                elif command.startswith("BR"):
                    if prevBR:
                        self.stm_sendq.put("bd090")
                        prevBR = False
                    else:
                        prevBR = True
                        self.insertCommand(["sx004","BR00","wx002"])
                        self.inserting.wait()
                        self.inserting.clear()
                        self.dupStates(1,2)
                        self.duplicating.wait()
                        self.duplicating.clear()
                        self.movement_lock.release()

                ## Others
                elif command.startswith("TP"):
                    self.rpi_queue.put(command)
                elif command.startswith("FIN"):
                    self.movement_lock.release()
                    logging.info("path ended")
                    sleep(5)
                    self.android_sendq.put("ROBOT END")
                    
                    url = f"http://{api_ip}:5000/stitch-image"
                    response = requests.get(url)

                    while True:
                        if self.android_sendq.empty() and self.stm_sendq.empty() and self.rpi_queue.empty():
                            sleep(5)
                            self.stop()

                # task 2
                elif command == "T2START":
                    self.STM.send("wx150")
                    if self.movement_lock.locked():
                        self.stm_sendq.put("T2SHORT")
                elif command == "T2SHORT":
                    self.rpi_queue.put("T2SHORTPIC")
                    self.arrow_recog.wait()
                    self.arrow_recog.clear()
                elif command == "T2LONG":
                    self.rpi_queue.put("T2LONGPIC")
                    self.arrow_recog.wait()
                    self.arrow_recog.clear()
                elif command == "T2GOBACK":
                    pass
                    # commands to go back to carpark
                elif command.startswith("T2"):
                    self.STM.send(command[3:])   
                else:
                    logging.error("command not recognised: {}".format(command))
            except KeyboardInterrupt:
                break

    # FUNCTION - ask the RPI to do things (child process)
    def rpiTasks(self): 
        while True:
            # get task from queue
            try:
                try:
                    task = self.rpi_queue.get(timeout=0.5)     # get queue item every .5s 
                except queue.Empty:
                    continue

                # TASK - take picture
                ### task format:
                # (A) TPxx  - take picture
                # (B) PATH  - get path from algo
                # case A - take picture
                if task.startswith("TP"):
                    logging.debug("taking picture")
                    
                    # DEBUG - skip taking pictures
                    logging.debug("SKIPPING PICTURES")

                    obsNo = int(task[2:])
                    '''
                    # capturing image
                    with picamera.PiCamera() as camera:
                        camera.resolution = (640,640)
                        camera.start_preview()
                        sleep(2)
                        camera.capture(f"OBS{obsNo}.jpg")
                        camera.close()

                    # calling API
                    url = f"http://{api_ip}:5000/test-image"
                    with open(f"OBS{obsNo}.jpg", "rb") as f:
                        image = f.read()
                    payload = {
                        "image": base64.b64encode(image).decode('utf-8'),
                        "image_type": "jpg",
                        "obs": obsNo
                    }
                    headers = {"Content-type": "application/json", "Accept": "text/plain"}
                    response = requests.post(url, headers=headers, json=payload)

                    # parse id response
                    data = response.json()
                    print(data["id"]=="NIL")
                    if data["id"] != "NIL":
                        # msg format: OBS,obstacle_id,image_id
                        self.android_sendq.put("OBS,{},{}".format(obsNo, data["id"]))
                    else:
                        logging.error("API cannot detect image")
                        self.android_sendq.put("OBS,{},{}".format(obsNo, "X"))
                    '''
                    self.android_sendq.put("OBS,{},{}".format(obsNo, "D"))
                    # start moving again
                    self.send_pic.wait()
                    self.send_pic.clear()
                    self.movement_lock.release()
                # case B - get path
                elif task.startswith("PATH"):
                    logging.debug("getting path")
                    # calling API
                    obs = []
                    while not self.obs_queue.empty():
                        obs.append(self.obs_queue.get())
                    url = f"http://{api_ip}:5000/get-path"
                    payload = {
                        "robotPos": robotPos,
                        "obs": obs
                    }
                    headers = {"Content-type": "application/json", "Accept": "text/plain"}
                    response = requests.post(url, headers=headers, json=payload)

                    # parse path response
                    data = response.json()
                    commands = data["path"]
                    states = data["states"]
                    for command in commands:
                        self.commandq.put(command)
                    self.commandq.put("FIN")
                    for state in states[1:]:
                        self.state_queue.put(state)
                    self.path_obtained.set()
                
                elif task.startswith("T2"):
                    if task == "T2SHORTPIC":
                        # capturing image
                        with picamera.PiCamera() as camera:
                            camera.resolution = (640,640)
                            camera.start_preview()
                            sleep(2)
                            camera.capture(f"T2short.jpg")
                            camera.close()

                        # calling API
                        url = f"http://{api_ip}:5000/test-image"
                        with open("T2short.jpg", "rb") as f:
                            image = f.read()
                        payload = {
                            "image": base64.b64encode(image).decode('utf-8'),
                            "image_type": "jpg",
                            "obs": 0    # not important
                        }
                        headers = {"Content-type": "application/json", "Accept": "text/plain"}
                        response = requests.post(url, headers=headers, json=payload)

                        # parse id response
                        data = response.json()
                        if data["id"] != "NIL":
                            if data["id"] == 38: #right
                                self.commandq.put()    # all the commands for turning right
                            elif data["id"] == 39: # left
                                self.commandq.put()    # all the commands for turning left
                            self.commandq.put("T2LONG")
                        else:
                            logging.info("API cannot detect image")

                        
                        # start moving again
                        self.arrow_recog.set()
                        self.movement_lock.release()

                    elif task == "T2LONGPIC":
                        # capturing image
                        with picamera.PiCamera() as camera:
                            camera.resolution = (640,640)
                            camera.start_preview()
                            sleep(2)
                            camera.capture(f"T2long.jpg")
                            camera.close()

                        # calling API
                        url = f"http://{api_ip}:5000/test-image"
                        with open("T2long.jpg", "rb") as f:
                            image = f.read()
                        payload = {
                            "image": base64.b64encode(image).decode('utf-8'),
                            "image_type": "jpg",
                            "obs": 0    # not important
                        }
                        headers = {"Content-type": "application/json", "Accept": "text/plain"}
                        response = requests.post(url, headers=headers, json=payload)

                        # parse id response
                        data = response.json()
                        if data["id"] != "NIL":
                            if data["id"] == "38 long": #right
                                self.commandq.put()    # all the commands for turning right
                            elif data["id"] == "39 long" : # left
                                self.commandq.put()    # all the commands for turning left
                            self.commandq.put("T2GOBACK")
                        else:
                            logging.error("API cannot detect image")
                        
                        # start moving again
                        self.arrow_recog.set()
                        self.movement_lock.release()
            except KeyboardInterrupt:
                break

    def insertCommand(self, commands):
        logging.debug("inserting commands")
        while not self.commandq.empty():
            command = self.commandq.get()
            commands.append(command)
        # logging.debug(commands)
        for command in commands:
            self.commandq.put(command)
        self.inserting.set()

    def dupStates(self, prev, next):
        logging.debug("duplicating states")
        states = []
        first = True
        while not self.state_queue.empty():
            state = self.state_queue.get()
            if first:
                for i in range(prev):
                    states.append("KEEP")
                for i in range(next):
                    states.append(state)
                first = False
            else:
                states.append(state)
        for state in states:
            self.state_queue.put(state)
        self.duplicating.set()

                    
if __name__ == "__main__":
    logging.basicConfig(level=logLevel,format="[%(levelname)s] %(funcName)s - %(message)s")
    controller = Brain()
    controller.run()