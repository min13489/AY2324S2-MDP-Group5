# if need to draw bounding box using CR model because detect.py used

import os, subprocess

def runCommands(commands):
    with open(os.devnull, 'wb') as devnull:
        for command in commands:
            subprocess.run(args=command.split(' '), shell=True, stdout=devnull, stderr=devnull)

# test
if __name__ == "__main__":
    # runCommands(["dir"])
    commands = []
    commands.append("py yolov5-master\detect.py --weights cr_2_a.pt --source ..\images\\20240216110339.jpg")
    obsNo = 1
    commands.append("ren ..\images\\boxed\\20240216110339.jpg OBS1_id1.jpg")
    runCommands(commands)