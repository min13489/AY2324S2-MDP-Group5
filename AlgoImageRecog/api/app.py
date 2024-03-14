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


### CODES ###

# SETUP - api
app = Flask(__name__)
app.config['DEBUG'] = True
CORS(app)

# SETUP - image recog
modelMode = True   # True for Yen, False for CR
if modelMode:
    yolo = YOLO("yen_3_a.pt", )

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
        print(states)
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
                results = yolo.predict(source="../images/{}.jpg".format(recvDateTime), verbose=False, device='cpu')
                # results = yolo("../images/{}.jpg".format(recvDateTime))

                # if no issues with prediction
                if results:
                    firstBox = results[0].boxes[0]                  # get first result
                    firstClass = yolo.names[int(firstBox.cls)]      # get id
                    firstConf = float(firstBox.conf)                # get conf
                    print("\n[INFO] test-image: predicted {} with confidence of {:.2f}".format(firstClass, firstConf))

                    id = firstClass[2:]     # remove 'id' prefix

                    # draw the bounding box
                    labels = ["{} {:.2f}".format(firstClass, firstConf)]            # label with id and conf
                    detections = sv.Detections.from_ultralytics(results[0])[0]      # for drawing of box - only consider first one
                    image = cv2.imread("../images/{}.jpg".format(recvDateTime))     # actual image
                    bounding_box_annotator = sv.BoundingBoxAnnotator()
                    label_annotator = sv.LabelAnnotator()
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
            return jsonify({"id": id})
        else:
            return jsonify({"id": "NIL"})

@app.route('/stitch-image', methods=["GET"])
def stitchImage():
    if request.method == "GET":
        if os.path.exists("../images/boxed/stitched.jpg"):
            os.remove("../images/boxed/stitched.jpg")

        img_paths = glob.glob("../images/boxed/OBS*.jpg")
        images = [Image.open(x) for x in img_paths]
        width, height = zip(*(i.size for i in images))
        total_width = sum(width)
        max_height = max(height)

        stitched_image = Image.new('RGB', (total_width, max_height))
        x_offset = 0

        for im in images:
            stitched_image.paste(im, (x_offset,0))
            x_offset += im.size[0]
        
        stitched_path = "..\images\\boxed\stitched.jpg"
        stitched_image.save(stitched_path)
        return "stitched\n"

if __name__ == "__main__":
    app.run("0.0.0.0", 5000) # run on all interfaces