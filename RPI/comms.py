### IMPORTS ###

# general
from time import sleep, time
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

obstacleCourse = 0           # 1 - obstacle course 0 - fastest car
# logLevel = logging.INFO      # logging.INFO - normal run
logLevel = logging.DEBUG     # logging.DEBUG - for debug msgs

### GLOBALS ###

robotPos = [2,2,'N']            # occupying bottom left corner (9 squares)
# api_ip = "192.168.5.29"         # min's computer IP
api_ip = "192.168.5.22"         # yen's computer IP
# api_ip = "192.168.5.23"         # dext's computer IP

### CODES ###

# SETUP - android connection class
class AndroidBT:
    # Initialisation
    def __init__(self):
        self.client_sock = None
        self.server_sock = None

    # Connect to Android device
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
            port = self.server_sock.getsockname()[1]
            logging.info("waiting for bluetooth connection on RFCOMM channel {}".format(port))

            # important for app connection
            uuid = "00001101-0000-1000-8000-00805F9B34FB"

            # receiving incoming connection
            self.client_sock, client_info = self.server_sock.accept()
            logging.info("accepted conenction from {}".format(client_info))

            return True # connection established
        except:
            logging.error("bluetooth connection unsuccessful")

            # cleanup
            if self.server_sock is not None:
                self.server_sock.close()
            if self.client_sock is not None:
                self.client_sock.close()
            self.server_sock = None
            self.client_sock = None
            return False # connection not established
    
    # Disconnect from Android device
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

    # Send message to Android device
    def send(self, msg):
        try:
            self.client_sock.send(msg.encode("utf-8"))
            logging.debug("RPI ---> ANDR: {}".format(msg))
        except:
            logging.error("RPI -/-> ANDR: {}".format(msg))

    # Receive message from Android device
    def receive(self):
        try:
            msg = self.client_sock.recv(1024)
            if msg is not None:
                logging.debug("ANDR ---> RPI: {}".format(msg.strip().decode("utf-8")))
                return msg.strip().decode("utf-8")
        except KeyboardInterrupt:
            pass
        except:
            logging.error("ANDR -/-> RPI")

# SETUP - stm connection class
class STMSerial:
    # Initialisation
    def __init__(self):
        self.serial_link = None

    # Connect to STM board
    def connect(self):
        logging.debug("establishing STM connection")

        # get the ttyUSB in use
        ttyUSB = [filename for filename in os.listdir("/dev") if filename.startswith("ttyUSB")]
        if len(ttyUSB) == 0:
            logging.error("serial not connected")
            return False # connection not established
        
        # connect
        self.serial_link = serial.Serial("/dev/{}".format(ttyUSB[0]), 115200)
        logging.info("serial connected")

        return True # connection established

    # Disconnect from STM board
    def disconnect(self):
        if self.serial_link is None:
            return
        self.serial_link.close()
        self.serial_link = None
        logging.info("serial disconnected")

    # Send message to STM board
    def send(self, msg):
        self.serial_link.write(msg.strip().encode(encoding="ascii"))
        logging.debug("RPI ---> STM: {}".format(msg))

    # Receive message from STM board
    def receive(self):
        try:
            msg = self.serial_link.read(3)
            if msg is not None:
                logging.debug("STM ---> RPI: {}".format(msg.strip().decode(encoding="ascii")))
                return msg.strip().decode(encoding="ascii")
        except:
            logging.error("STM -/-> RPI")
            pass
        
# SETUP - main controller class
class Brain:
    # Set up connection and multiprocessing
    def __init__(self):
        # general setup
        self.android = AndroidBT()                      # object - ANDR connection
        self.STM = STMSerial()                          # object - STM connection
        self.manager = Manager()                        # manager - multiprocessing
        self.android_sendq = self.manager.Queue()       # queue - things to send to ANDR
        self.stm_sendq = self.manager.Queue()           # queue - things to send to STM
        self.rpi_queue = self.manager.Queue()           # queue - things for RPI to do
        self.commandq = self.manager.Queue()            # queue - commands from PATH or custom
        self.proc_sendAndroid = None                    # process - sending messages to ANDR
        self.proc_recvAndroid = None                    # process - receiving messages from ANDR
        self.proc_sendSTM = None                        # process - sending messages to STM
        self.proc_recvSTM = None                        # process - receiving messages from STM
        self.proc_rpi = None                            # process - run RPI tasks
        self.proc_robotRun = None                       # process - read commands from queue
        self.movement_lock = self.manager.Lock()        # lock - per command
        self.android_dropped = self.manager.Event()     # event - Android disconnected

        # TASK 1 - specific setup
        if obstacleCourse:
            self.obs_queue = self.manager.Queue()       # queue - obs list
            self.state_queue = self.manager.Queue()     # queue - states list
            self.path_obtained = self.manager.Event()   # event - path obtained or not
            self.send_pic = self.manager.Event()        # event - image sent
            self.inserting = self.manager.Event()       # event - command insertion
            self.duplicating = self.manager.Event()     # event - state duplication
        # TASK 2 - specific setup
        else:
            self.arrow_recog = self.manager.Event()     # event - arrow recognised

    # Check is API is alive
    def pingAPI(self):
        output = subprocess.run(["curl", "http://{}:5000/".format(api_ip)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL).stdout.strip().decode("utf-8")
        if output == "API is up and running":
            logging.info("API up")
            return True # API up
        else:
            logging.error("API down, check if connected to subnet")
            return False # API down

    # Start connections and run multiprocessing
    def run(self):
        try:
            # establish connections
            conn1, conn2, conn3 = False, False, False
            conn1 = self.android.connect()
            if conn1:
                self.android_sendq.put("CONNECTED TO RPI")
            conn2 = self.STM.connect()
            conn3 = self.pingAPI()
            
            if not (conn1 and conn2 and conn3):
                self.android.disconnect()
                self.STM.disconnect()
                logging.error("some connection not properly established. aborted")
                self.stop() # exit program if connection not ready
                return
            logging.info("all connections ready")

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
            self.reconnect_android()                # waiting and test for disconnection

        except KeyboardInterrupt:
            print("\r>>> MANUAL KILL PROGRAM <<<")
            self.stop()
    
    # Stop the run
    def stop(self):
        self.android.disconnect()
        self.STM.disconnect()
        try:
            self.proc_sendAndroid.kill()
            self.proc_recvAndroid.kill()
            self.proc_sendSTM.kill()
            self.proc_recvSTM.kill()
            self.proc_rpi.kill()
        except:
            pass
        logging.info("program exit")

    # Monitor for android connection dropped
    ### BUG: Doesn't seem to be working but whatever
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
    
    # Child process that sends messages to ANDR
    def sendAndroid(self):
        while True:
            # get message from queue
            try:
               msg = self.android_sendq.get(timeout=0.5)
            except queue.Empty:
                continue
            
            # send message
            try:
                self.android.send(msg)
                if msg.startswith("OBS"):   # indicate that pic sent
                    self.send_pic.set()
            except OSError:
                self.android_dropped.set()

    # Child process that receives messages from ANDR     
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
            msg_parts = msg_str.split(",")

            ### case A - insertion
            if msg_parts[0] == "OBS":
                obs_item = [20-int(msg_parts[2]), int(msg_parts[3])+1, msg_parts[4], int(msg_parts[1])]
                self.obs_queue.put(obs_item)
                logging.info("obstacle inserted - {}".format(str(obs_item)))

            ### case B - removal
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

            ### case C - start
            elif msg_parts[0] == "START":
                if not self.pingAPI():
                    logging.error("API down cannot start path")
                    self.android_sendq.put("API down, cannot start")
                if obstacleCourse and self.obs_queue.empty():   # TASK 1 - requires obs_queue to have something
                    logging.error("no obstacles cannot start path")
                    self.android_sendq.put("No obstacles")
                else:
                    if obstacleCourse:  # TASK 1 - requires path from Algo
                        self.rpi_queue.put("TIMER")
                        self.rpi_queue.put("PATH")
                        self.path_obtained.wait()
                        self.android_sendq.put("MOVING")
                    else:
                        self.commandq.put("T2START")
        
    # Child process that sends messages to STM
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
    
    # Child process that receives messages from STM
    def recvSTM(self):
        while True:
            try:
                msg = self.STM.receive()
                # normal ACK
                if msg.startswith("ACK"):
                    try:
                        if obstacleCourse:
                            currentPos = self.state_queue.get()
                            if currentPos != "KEEP":
                                self.android_sendq.put("ROBOT,{},{},{}".format(str(20-currentPos[0]),str(currentPos[1]-1),currentPos[2]))
                        # sleep(2) # check if this affects
                        self.movement_lock.release()
                    except RunTimeError:
                        logging.error("tried to release a released lock")
                # BMP when using US
                elif msg.startswith("BMP") and not obstacleCourse:
                    logging.debug("BMP from ultrasonic sensor")
                    try:
                        sleep(2)
                        self.movement_lock.release()
                    except RuntimeError:
                        logging.error("tried to release a released lock")
            except KeyboardInterrupt:
                break

    # Child process that runs the robot using commands issued
    def robotRun(self):
        # in case we need to insert commands for turning
        prevFL = False
        prevFR = False
        prevBL = False
        prevBR = False
        # task ended flag
        ended = False

        while not ended:
            try:
                while not self.commandq.empty():
                    # get command
                    command = self.commandq.get(timeout=0.5)    # get queue item every .5s
                    
                    # acquire movement lock
                    self.movement_lock.acquire()
                    
                    # DEBUG
                    # logging.debug("note issue and move to by right location")
                    # sleep(2)

                    # process command
                    logging.debug(command)

                    ### FW and BW movements
                    if command.startswith("FW"):
                        distance = int(command[2:])
                        send_str = "wx{:03d}".format(distance*10)
                        self.stm_sendq.put(send_str)
                    elif command.startswith("BW"):
                        distance = int(command[2:])
                        send_str = "sx{:03d}".format(distance*10)
                        self.stm_sendq.put(send_str)

                    ## TURN movements
                    elif command.startswith("FL"):
                        if prevFL or not obstacleCourse:
                            self.stm_sendq.put("fa092")
                            prevFL = False
                        # TASK 1 custom to stay in the 30x30 square
                        else:
                            prevFL = True
                            self.insertCommand(["sx002","FL00","sx005"])
                            self.inserting.wait()
                            self.inserting.clear()
                            self.dupStates(1,2) # keep at previous for 1, keep at next for 2
                            self.duplicating.wait()
                            self.duplicating.clear()
                            self.movement_lock.release()
                    elif command.startswith("FR"):
                        if prevFR or not obstacleCourse:
                            self.stm_sendq.put("fd085") # Yen change
                            prevFR = False
                        # TASK 1 custom to stay in the 30x30 square
                        else:
                            prevFR = True
                            self.insertCommand(["wx003","FR00"])
                            self.inserting.wait()
                            self.inserting.clear()
                            self.dupStates(1,1) # keep at previous for 1, keep at next for 1
                            self.duplicating.wait()
                            self.duplicating.clear()
                            self.movement_lock.release()
                    elif command.startswith("BL"):
                        if prevBL or not obstacleCourse:
                            self.stm_sendq.put("ba091") # Yen change from 90 to 85
                            prevBL = False
                        # TASK 1 custom to stay in the 30x30 square
                        else:
                            prevBL = True
                            self.insertCommand(["BL00","sx003"])
                            self.inserting.wait()
                            self.inserting.clear()
                            self.dupStates(0,2) # don't keep at previous, keep at next for 2
                            self.duplicating.wait()
                            self.duplicating.clear()
                            self.movement_lock.release()
                    elif command.startswith("BR"):
                        if prevBR or not obstacleCourse:
                            # self.stm_sendq.put("bd090")
                            self.stm_sendq.put("bd085") # Yen change from 85 to 80
                            prevBR = False
                        # TASK 1 custom to stay in the 30x30 square
                        else:
                            prevBR = True
                            self.insertCommand(["sx003","BR00","sx004"])
                            self.inserting.wait()
                            self.inserting.clear()
                            self.dupStates(1,2)
                            self.duplicating.wait()
                            self.duplicating.clear()
                            self.movement_lock.release()

                    ### Others
                    elif command.startswith("TP"):
                        self.rpi_queue.put(command)
                    elif command.startswith("FIN"):
                        self.rpi_queue.put("STITCH")
                        self.movement_lock.release()
                        logging.info("run ended")
                        sleep(5)
                        self.android_sendq.put("ROBOT END")

                        # wait for all queues to clear
                        while not (self.android_sendq.empty() and self.stm_sendq.empty() and self.rpi_queue.empty()):
                            continue
                        ended = True

                    # TASK 2 commands
                    elif command == "T2START":
                        self.stm_sendq.put("wz150")             # ultrasonic - interrupt @ 40, stop at 30
                        self.commandq.put("T2SHORT")
                    elif command == "T2SHORT":
                        self.rpi_queue.put("T2SHORTPIC")
                        self.arrow_recog.wait()
                        self.arrow_recog.clear()
                    elif command == "T2TOLONG":
                        self.stm_sendq.put("wz150")             # ultrasonic - interrupt @ 40, stop at 30
                        # self.stm_sendq.put("sz150")  
                        if self.movement_lock.acquire():
                            self.movement_lock.release()
                            # self.stm_sendq.put("sz150")         # ultrasonic - interrupt @ 40, stop at 50
                            # self.stm_sendq.put("sx020")
                            self.commandq.put("T2LONG")
                    elif command == "T2LONG":
                        self.rpi_queue.put("T2LONGPIC")
                        self.arrow_recog.wait()
                        self.arrow_recog.clear()
                    elif command == "T2GOBACK":
                        # commands
                        self.stm_sendq.put("ALLAH")
                        logging.info("by right should have gone back here but waiting for new commands")
                        self.commandq.put("FIN")
                    
                    # all other commands (direct STM)
                    else:
                        self.stm_sendq.put(command)
                        logging.debug("custom command: {}".format(command))
            except KeyboardInterrupt:
                break
        
        self.stop()

    # Child process that processes tasks for RPI, mainly API calls
    def rpiTasks(self): 
        # counting the time used for the task
        timeStart = None
        timeEnd = None
        # retry for straight-right deviation
        # retry = True

        while True:
            try:
                while not self.rpi_queue.empty():
                    # get task
                    task = self.rpi_queue.get(timeout=0.5)

                    # execute task
                    logging.debug(task)

                    ### case A - start timer
                    if task.startswith("TIMER"):
                        timeStart = time()

                    ### case B - get path (TASK 1)
                    elif task.startswith("PATH"):
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
                    
                    ### case C - take picture (TASK 1)
                    elif task.startswith("TP"):
                        # DEBUG - skip taking pictures
                        # logging.debug("SKIPPING PICTURES")

                        obsNo = int(task[2:])
                        
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
                        if data["id"] != "NIL":
                            # msg format: OBS,obstacle_id,image_id
                            self.android_sendq.put("OBS,{},{}".format(obsNo, data["id"]))
                        else:
                            logging.error("API cannot detect image")
                            
                            # TODO: see if anyway to only run this if the distance moved more than XX
                            # BUG
                            # print(retry and obstacleCourse)
                            # if retry and obstacleCourse:
                            #     self.insertCommand(['sx010', 'fa005', 'wx008', task]) ## TOTEST
                            #     self.inserting.wait()
                            #     self.inserting.clear()
                            #     self.dupStates(3,0)
                            #     self.duplicating.wait()
                            #     self.duplicating.clear()
                            #     retry = False
                            # else:
                            #     retry = True
                            #     self.android_sendq.put("OBS,{},{}".format(obsNo, "X"))
                            self.android_sendq.put("OBS,{},{}".format(obsNo, "X"))
                        
                        # DEBUG
                        # self.android_sendq.put("OBS,{},{}".format(obsNo, "D"))

                        # start moving again
                        self.send_pic.wait()
                        self.send_pic.clear()
                        self.movement_lock.release()

                    ### case D: stitch images
                    elif task.startswith("STITCH"):
                        timeEnd = time()
                        timeTaken = int(timeEnd - timeStart)
                        logging.info("time used for run: {}m {}s".format(timeTaken/60, timeTaken%60))
                        url = f"http://{api_ip}:5000/stitch-image"
                        requests.get(url)
                        logging.info("stitching completed")

                    ### case E: first obstacle picture (TASK 2)
                    elif task.startswith("T2SHORTPIC"):
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
                            "obs": 1    # not important
                        }
                        headers = {"Content-type": "application/json", "Accept": "text/plain"}
                        response = requests.post(url, headers=headers, json=payload)

                        # parse id response
                        data = response.json()
                        if data["id"] != "NIL":
                            if data["id"] == "38": # right
                                self.commandq.put("fd050")
                                self.commandq.put("fa110")
                                self.commandq.put("fd050")
                                self.commandq.put("sx020")#
                            elif data["id"] == "39": # left
                                # self.commandq.put("sx005") # Yen testing
                                self.commandq.put("fa060")
                                self.commandq.put("fd100")
                                self.commandq.put("fa053")
                                self.commandq.put("sx020")#
                            self.commandq.put("T2TOLONG")
                        else:
                            logging.info("API cannot detect image")

                        # start moving again
                        self.arrow_recog.set()
                        self.movement_lock.release()

                    ### case F: second obstacle picture (TASK 2)
                    elif task.startswith("T2LONGPIC"):
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
                            "obs": 2    # not important
                        }
                        headers = {"Content-type": "application/json", "Accept": "text/plain"}
                        response = requests.post(url, headers=headers, json=payload)

                        # parse id response
                        data = response.json()
                        if data["id"] != "NIL":
                            if data["id"] == "38" or data["id"] == "38 long": #right
                                # TODO
                                # self.commandq.put()    # all the commands for turning right
                                
                                # this is correct ones
                                self.commandq.put("fd090")
                                self.commandq.put("sx030")
                                self.commandq.put("wi150")
                                self.commandq.put("sx015")
                                self.commandq.put("fa090")
                                self.commandq.put("fd000")
                                self.commandq.put("sx005")
                                self.commandq.put("fa090")
                                self.commandq.put("fd000")
                                # self.commandq.put("wx020") ## Yen added testing
                                self.commandq.put("wi150")
                                self.commandq.put("sx005")
                                self.commandq.put("fa090")
                                self.commandq.put("fd000")
                                """
                                # This is testing ones
                                self.commandq.put("fa090")
                                self.commandq.put("fd000")
                                self.commandq.put("sx020")
                                self.commandq.put("wi150")
                                self.commandq.put("fd093")
                                # self.commandq.put("sx005")
                                self.commandq.put("fd093")
                                self.commandq.put("fa000")
                                self.commandq.put("wi200")
                                # self.commandq.put("sx005")
                                self.commandq.put("fd090")
                                """
                            elif data["id"] == "39" or data["id"] == "39 long" : # left
                                #
                                # self.commandq.put()    # all the commands for turning left
                                self.commandq.put("fa090")
                                self.commandq.put("fd000")
                                self.commandq.put("sx020")
                                self.commandq.put("wi150")
                                self.commandq.put("fd090")
                                self.commandq.put("sx005")
                                self.commandq.put("fd090")
                                self.commandq.put("fa000")
                                self.commandq.put("wi200")
                                self.commandq.put("sx005")
                                self.commandq.put("fd090")
                            self.commandq.put("T2GOBACK") # TODO
                        else:
                            logging.error("API cannot detect image")
                        
                        # start moving again
                        self.arrow_recog.set()
                        self.movement_lock.release()
            except KeyboardInterrupt:
                break
    
    # Helper function to insert commands to front of queue
    def insertCommand(self, commands):
        logging.debug("inserting commands")
        while not self.commandq.empty():
            command = self.commandq.get()
            commands.append(command)
        # logging.debug(commands)
        for command in commands:
            self.commandq.put(command)
        self.inserting.set()

    # Helper function to duplicate the states of the robot for TASK 1
    def dupStates(self, prev, next):
        logging.debug("duplicating states")
        states = []
        first = True
        while not self.state_queue.empty():
            state = self.state_queue.get()
            if first:
                for i in range(prev):
                    states.append("KEEP")
                if next > 0:
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
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
