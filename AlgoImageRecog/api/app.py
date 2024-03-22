### IMPORTS ###

# api server requirements
from flask import Flask, request, jsonify
from flask_cors import CORS
# algo
from algo import generatePath
# image recog
from datetime import datetime
import base64, glob
from PIL import Image
# yen model
from ultralytics import YOLO
import supervision as sv
import cv2
# cr model
import pathlib, torch
from parallel import runCommands
from multiprocessing import Process
# general
import os
from math import ceil

### CODES ###

# SETUP - api
app = Flask(__name__)
app.config['DEBUG'] = True
CORS(app)

# SETUP - image recog
modelMode = True   # True for Yen, False for CR
if modelMode:
    # yolo = YOLO("yen_3_a.pt")
    yolo = YOLO("yen_2_b.pt")
imageNameMap = {
    "11": "Number 1",
    "12": "Number 2",
    "13": "Number 3",
    "14": "Number 4",
    "15": "Number 5",
    "16": "Number 6",
    "17": "Number 7",
    "18": "Number 8",
    "19": "Number 9",
    "20": "Alphabet A",
    "21": "Alphabet B",
    "22": "Alphabet C",
    "23": "Alphabet D",
    "24": "Alphabet E",
    "25": "Alphabet F",
    "26": "Alphabet G",
    "27": "Alphabet H",
    "28": "Alphabet S",
    "29": "Alphabet T",
    "30": "Alphabet U",
    "31": "Alphabet V",
    "32": "Alphabet W",
    "33": "Alphabet X",
    "34": "Alphabet Y",
    "35": "Alphabet Z",
    "36": "Up arrow",
    "37": "Down arrow",
    "38": "Right arrow",
    "39": "Left arrow",
    "40": "Stop",
    "99": "Bull's eye"
}

def resetEnv():
    fileList = os.listdir("../images/boxed")
    for file in fileList:
        if file.startswith("OBS"):
            os.remove("../images/boxed/{}".format(file))

# ROUTE - to test if API is up
@app.route("/")
def hello():
    resetEnv()
    return "API is up and running\n"

# ROUTE - to retrieve path from algo
@app.route("/get-path", methods=["POST"])
def getPath():
    if request.method == "POST" and request.is_json:
        data = request.get_json()
        path, states = generatePath(data["robotPos"], data["obs"])
        # print(states)
        print(path)
        # path = ["BW03", "BR00", "BW04", "BL00"]
        return jsonify({"path": path, "states": states}), 200

# ROUTE - to retrieve id from image recog
@app.route('/test-image', methods=["POST"])
def testImage():
    if request.method == "POST" and request.is_json:
        # get image
        data = request.get_json()
        recvDateTime = datetime.now().strftime("%Y%m%d%H%M%S")
        if data["image_type"] == "jpg":
            image = base64.b64decode(data["image"].encode('utf-8'))
            with open("../images/{}.jpg".format(recvDateTime), "wb") as f:
                f.write(image)
        
        # send image into model
        id = None
        if modelMode:   # yen's model
            try:
                # predict the id of image
                numPredictions = 0
                results = yolo.predict(source="../images/{}.jpg".format(recvDateTime), verbose=False)
                # results = yolo("../images/{}.jpg".format(recvDateTime))

                # if no issues with prediction
                if results:
                    largest_area = 0
                    largest_area_index = 0
                    for result_index in range(len(results)):
                        box = results[result_index].boxes      
                        area = (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1])
                        if area > largest_area:
                            largest_area = area
                            largest_area_index = result_index
                            
                    numPredictions = len(results)                   # get number of result found
                    firstBox = results[largest_area_index].boxes[0]                  # get first result
                    firstClass = yolo.names[int(firstBox.cls)]      # get id
                    firstConf = float(firstBox.conf)                # get conf
                    print("\n[INFO] test-image: predicted {} with confidence of {:.2f}".format(firstClass, firstConf))

                    id = firstClass[2:]     # remove 'id' prefix

                    # draw the bounding box
                    labels = ["{}, id={}".format(imageNameMap[id], id)]            # label with name and id
                    detections = sv.Detections.from_ultralytics(results[largest_area_index])[0]     # for drawing of box - only consider first one
                    image = cv2.imread("../images/{}.jpg".format(recvDateTime))    # actual image
                    bounding_box_annotator = sv.BoundingBoxAnnotator()
                    label_annotator = sv.LabelAnnotator(text_position=sv.geometry.core.Position.TOP_LEFT_CUSTOM)
                    annotated_image = bounding_box_annotator.annotate(scene=image, detections=detections)                               # draw box
                    annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)             # labels on the box
                    cv2.imwrite("../images/boxed/OBS{}_id{}.jpg".format(data["obs"],id if id is not None else 'x'), annotated_image)    # save the annotated image

            # if there is an error trying to detect the image, it will come here
            except IndexError:
                print("\n[ERROR] test-image: couldn't detect the image")
                id = None # might want to customise this to handle this case

        else: # cr model
            tempcgr = pathlib.PosixPath
            pathlib.PosixPath = pathlib.WindowsPath
            model = torch.hub.load('ultralytics/yolov5', 'custom', path='cr_2_a.pt')

            results = model("../images/{}.jpg".format(recvDateTime))

            resultsdf = results.pandas().xyxy[0]
            # print(resultsdf)
            print("\nPREDICTION: {}".format(resultsdf["name"].iloc[0]))

            pathlib.PosixPath = tempcgr

            id = str(resultsdf["name"].iloc[0])[2:]

            commands = []
            commands.append(f"py yolov5-master\detect.py --weights cr_2_a.pt --source ..\images\{recvDateTime}.jpg")
            obsNo = data["obs"]
            commands.append(f"ren ..\images\\boxed\{recvDateTime}.jpg OBS{obsNo}_id{id if id is not None else 'x'}.jpg")

            parallel_proc = Process(target=runCommands, args=[commands])
            parallel_proc.start()

        # send id back
        if id is not None:
            return jsonify({"num_predictions": numPredictions, "id": id})
        else:
            return jsonify({"num_predictions": numPredictions, "id": "NIL"})

@app.route('/stitch-image', methods=["GET"])
def stitchImage():
    if request.method == "GET":
        # remove previously stitched image, if any
        if os.path.exists("../images/boxed/stitched.jpg"):
            os.remove("../images/boxed/stitched.jpg")

        # retrieve all OBS images
        img_paths = glob.glob("../images/boxed/OBS*.jpg")

        # if no images to stitch
        if len(img_paths) == 0: 
            return "nothing to stitch\n"

        # stitching
        images = [Image.open(x) for x in img_paths]
        perRow = ceil(len(images) / 2)
        # width, height = zip(*(i.size for i in images))  # all are 640 x 640
        total_width = perRow * 640      # instead of sum(width)
        max_height = 2 * 640            # instead of max(height)

        stitched_image = Image.new('RGB', (total_width, max_height))
        x_offset = 0
        y_offset = 0

        rowCount = 0
        for im in images:
            if rowCount == perRow:
                x_offset = 0
                y_offset = 640
                rowCount = 0
            stitched_image.paste(im, (x_offset,y_offset))
            x_offset += 640        # instead of im.size[0] because fixed
            rowCount += 1
        
        while x_offset != total_width:
            placeholder = Image.open("../images/boxed/placeholder.jpg")
            stitched_image.paste(placeholder, (x_offset, y_offset))
            x_offset += 640
        
        stitched_path = "../images/boxed/stitched.jpg"
        stitched_image.save(stitched_path)
        return "stitched\n"

if __name__ == "__main__":
    app.run("0.0.0.0", 5000) # run on all interfaces