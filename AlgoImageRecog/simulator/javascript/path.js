var countDownTime = null
var shortenFactor = 1
var stopRobot = false

function pathSetup() {

    // remove the div-setup
    var setupDiv = document.getElementById("div-setup");
    setupDiv.parentElement.removeChild(setupDiv);

    // add elements and display div-path
    var pathDiv = document.getElementById("div-path");

        // path instruction p
    var pathInstruct = document.createElement("p");
    pathInstruct.setAttribute("id","path-instruct");
    var pathInstructText = document.createTextNode("Path is ready.");
    pathInstruct.appendChild(pathInstructText);
    pathDiv.appendChild(pathInstruct);

        // countdown p
    var countDownPara = document.createElement("p");
    countDownPara.setAttribute("id","countdown-text");
    var countDownText = document.createTextNode("Time left: ")
    countDownPara.appendChild(countDownText)
    countDownTime = document.createElement("span");
    var minutes = (Math.trunc(countDown / 60)).toString().padStart(2,'0');
    var seconds = (countDown % 60).toString().padStart(2,'0');
    countDownTime.innerHTML = `${minutes}:${seconds}`
    countDownPara.appendChild(countDownTime);
    pathDiv.appendChild(countDownPara);
    
        // button div
    var playDiv = document.createElement("div");
    playDiv.setAttribute("id", "div-play");
    
        // button
    var playButton = document.createElement("button");
    playButton.setAttribute("id", "button-play");
    playButton.textContent = "Play";
    playDiv.appendChild(playButton)
    pathDiv.appendChild(playDiv);
    playButton.addEventListener("click", () => runPath());
        // show
    pathDiv.style.display = "block";

        // rmb original pos
    originalRobotPos = [...robotPos];
}

// sleep function
const sleep = (delay) => new Promise((resolve) => setTimeout(resolve, delay));

var countDownMaster = 120;
var countDown = countDownMaster;
var countDownId = null;

function countDownTimer() {
    countDown--;
    if (countDown >= 0) {
        console.log(countDown);
        var minutes = (Math.trunc(countDown / 60)).toString().padStart(2,'0');
        var seconds = (countDown % 60).toString().padStart(2,'0');
        countDownTime.innerHTML = `${minutes}:${seconds}`
    } else {
        clearInterval(countDownId);
        stopRobot = true;
    }
}

async function runPath() {
    playButton = document.getElementById("button-play");
    playButton.disabled = true;
    playButton.id = "button-noPlay";

    var countDown = countDownMaster;
    var minutes = (Math.trunc(countDown / 60)).toString().padStart(2,'0');
    var seconds = (countDown % 60).toString().padStart(2,'0');
    countDownTime.innerHTML = `${minutes}:${seconds}`;

    clearOld(robotPos);
    robotPos = [...originalRobotPos];
    colorNew("#0000FF", "#AAAAFF");
    await sleep(1000);

    countDownId = setInterval(countDownTimer, 1000*shortenFactor)

    for (var i = 0, step; step = pathArray[i]; i++) {
        if (stopRobot) {
            break;
        }

        // console.log(step);
        pathInstruct = document.getElementById("path-instruct");
        pathInstruct.innerHTML = `Current instruction: ${step}`;
        
        switch (step.substring(0,2)) {
            case "FW":
                var repeatFor = parseInt(step.substring(2));
                for (var j = 0; j < repeatFor; j++) {
                    await sleep(1000*shortenFactor);
                    if (stopRobot) {
                        break;
                    }
                    moveFront();
                }
                break;
            case "BW":
                var repeatFor = parseInt(step.substring(2));
                for (var j = 0; j < repeatFor; j++) {
                    await sleep(1000*shortenFactor);
                    if (stopRobot) {
                        break;
                    }
                    moveBack();
                }
                break;
            case "FL":
                await sleep(3000*shortenFactor);
                if (stopRobot) {
                    break;
                }
                moveFrontLeft();
                break;
            case "FR":
                await sleep(3000*shortenFactor);
                if (stopRobot) {
                    break;
                }
                moveFrontRight();
                break;
            case "BL":
                await sleep(3000*shortenFactor);
                if (stopRobot) {
                    break;
                }
                moveBackLeft();
                break;
            case "BR":
                await sleep(3000*shortenFactor);
                if (stopRobot) {
                    break;
                }
                moveBackRight();
                break;
            case "TP":
                await takePic();
                break;
        }
    }
    clearInterval(countDownId);
    
    await sleep(1000);

    playButton.id = "button-play";
    playButton.disabled = false;
}

// straight
function moveFront() {
    oldRobotPos = [...robotPos];
    if (robotPos[2] == 'N') {
        robotPos = [robotPos[0]+1,robotPos[1],robotPos[2]];
    } else if (robotPos[2] == 'E') {
        robotPos = [robotPos[0],robotPos[1]+1,robotPos[2]];
    } else if (robotPos[2] == 'S') {
        robotPos = [robotPos[0]-1,robotPos[1],robotPos[2]];
    } else {
        robotPos = [robotPos[0],robotPos[1]-1,robotPos[2]];
    }
    clearOld(oldRobotPos);
    colorNew("#0000FF", "#AAAAFF");
}

function moveBack() {
    oldRobotPos = [...robotPos];
    if (robotPos[2] == 'N') {
        robotPos = [robotPos[0]-1,robotPos[1],robotPos[2]];
    } else if (robotPos[2] == 'E') {
        robotPos = [robotPos[0],robotPos[1]-1,robotPos[2]];
    } else if (robotPos[2] == 'S') {
        robotPos = [robotPos[0]+1,robotPos[1],robotPos[2]];
    } else {
        robotPos = [robotPos[0],robotPos[1]+1,robotPos[2]];
    }
    clearOld(oldRobotPos);
    colorNew("#0000FF", "#AAAAFF");
}

// turn

function moveFrontLeft() {
    oldRobotPos = [...robotPos];
    if (robotPos[2] == 'N') {
        robotPos = [robotPos[0]+3,robotPos[1]-3,'W'];
    } else if (robotPos[2] == 'E') {
        robotPos = [robotPos[0]+3,robotPos[1]+3,'N'];
    } else if (robotPos[2] == 'S') {
        robotPos = [robotPos[0]-3,robotPos[1]+3,'E'];
    } else {
        robotPos = [robotPos[0]-3,robotPos[1]-3,'S'];
    }
    clearOld(oldRobotPos);
    colorNew("#0000FF", "#AAAAFF");
}

function moveBackLeft() {
    oldRobotPos = [...robotPos];
    if (robotPos[2] == 'N') {
        robotPos = [robotPos[0]-3,robotPos[1]-3,'E'];
    } else if (robotPos[2] == 'E') {
        robotPos = [robotPos[0]+3,robotPos[1]-3,'S'];
    } else if (robotPos[2] == 'S') {
        robotPos = [robotPos[0]+3,robotPos[1]+3,'W'];
    } else {
        robotPos = [robotPos[0]-3,robotPos[1]+3,'N'];
    }
    clearOld(oldRobotPos);
    colorNew("#0000FF", "#AAAAFF");
}

function moveFrontRight() {
    oldRobotPos = [...robotPos];
    if (robotPos[2] == 'N') {
        robotPos = [robotPos[0]+3,robotPos[1]+3,'E'];
    } else if (robotPos[2] == 'E') {
        robotPos = [robotPos[0]-3,robotPos[1]+3,'S'];
    } else if (robotPos[2] == 'S') {
        robotPos = [robotPos[0]-3,robotPos[1]-3,'W'];
    } else {
        robotPos = [robotPos[0]+3,robotPos[1]-3,'N'];
    }
    clearOld(oldRobotPos);
    colorNew("#0000FF", "#AAAAFF");
}

function moveBackRight() {
    oldRobotPos = [...robotPos];
    if (robotPos[2] == 'N') {
        robotPos = [robotPos[0]-3,robotPos[1]+3,'W'];
    } else if (robotPos[2] == 'E') {
        robotPos = [robotPos[0]-3,robotPos[1]-3,'N'];
    } else if (robotPos[2] == 'S') {
        robotPos = [robotPos[0]+3,robotPos[1]-3,'E'];
    } else {
        robotPos = [robotPos[0]+3,robotPos[1]+3,'S'];
    }
    clearOld(oldRobotPos);
    colorNew("#0000FF", "#AAAAFF");
}

// coloring and util

function clearOld(oldRobotPos) {
    var map = document.getElementById("table-map");
    let x = 20-oldRobotPos[0];
    let y = oldRobotPos[1];

    map.rows[x-1].cells[y-1].style.backgroundColor = "";
    map.rows[x-1].cells[y].style.backgroundColor = "";
    map.rows[x-1].cells[y+1].style.backgroundColor = "";

    map.rows[x].cells[y-1].style.backgroundColor = "";
    map.rows[x].cells[y].style.backgroundColor = "";
    map.rows[x].cells[y+1].style.backgroundColor = "";

    map.rows[x+1].cells[y-1].style.backgroundColor = "";
    map.rows[x+1].cells[y].style.backgroundColor = "";
    map.rows[x+1].cells[y+1].style.backgroundColor = "";
}

function colorNew(bg, head) {
    var map = document.getElementById("table-map");
    let x = 20-robotPos[0];
    let y = robotPos[1];
    let dir = robotPos[2];

    for (var row = x-1; row <= x+1; row++) {
        for (var col = y-1; col <= y+1; col++) {
            map.rows[row].cells[col].style.backgroundColor = bg;
        }
    }

    switch (dir) {
        case "N":
            map.rows[x-1].cells[y].style.backgroundColor = head;
            break;
        case "E":
            map.rows[x].cells[y+1].style.backgroundColor = head;
            break;
        case "S":
            map.rows[x+1].cells[y].style.backgroundColor = head;
            break;
        case "W":
            map.rows[x].cells[y-1].style.backgroundColor = head;
    }

}

async function takePic() {
    var map = document.getElementById("table-map");
    let x = 20-robotPos[0];
    let y = robotPos[1];
    let dir = robotPos[2];

    // goal reached colour
    colorNew("#005700", "#B5F7B5");

    await sleep(5000*shortenFactor); // capture, recognise, display
    if (stopRobot) {
        return Promise.resolve("");
    }
    // original colour 
    colorNew("#0000FF", "#AAAAFF");

    // to ensure full execution before next step
    return Promise.resolve("");
}