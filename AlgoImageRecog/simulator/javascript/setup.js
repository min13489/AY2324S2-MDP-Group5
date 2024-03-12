window.onload = () => stepOne();
var robotPos = [null,null,null];
var obs = [];
var stepCount = 3;

function stepOne() {
    var map = document.getElementById("table-map");
    document.getElementById("setup-instruct").innerHTML = "Step 1: Select robot position.";
    for (var i = 0, row; row = map.rows[i]; i++) {
        if (i > 0 & i < 19) {
            for (var j = 1, col; col = row.cells[j]; j++) {
                if (j > 1 & j < 20) {
                    (function(x,y) {
                        col.addEventListener("mouseenter", () => tryRobot(x,y));
                    })(i,j);
                    (function(x,y) {
                        col.addEventListener("mouseleave", () => unTryRobot(x,y));
                    })(i,j);
                    (function(x,y) {
                        col.addEventListener("click", () => clearRobotListeners(x,y));
                    })(i,j);
                }
            }
        }
    }
}

function tryRobot(x,y) {
    var map = document.getElementById("table-map");

    map.rows[x-1].cells[y-1].style.backgroundColor = "#0000FF50";
    map.rows[x-1].cells[y].style.backgroundColor = "#0000FF50";
    map.rows[x-1].cells[y+1].style.backgroundColor = "#0000FF50";

    map.rows[x].cells[y-1].style.backgroundColor = "#0000FF50";
    map.rows[x].cells[y].style.backgroundColor = "#0000FF50";
    map.rows[x].cells[y+1].style.backgroundColor = "#0000FF50";

    map.rows[x+1].cells[y-1].style.backgroundColor = "#0000FF50";
    map.rows[x+1].cells[y].style.backgroundColor = "#0000FF50";
    map.rows[x+1].cells[y+1].style.backgroundColor = "#0000FF50";
}

function unTryRobot(x,y) {
    var map = document.getElementById("table-map");

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

function clearRobotListeners(x,y) {
    var map = document.getElementById("table-map");
    var cloneMap = map.cloneNode(true);
    map.parentNode.replaceChild(cloneMap, map);

    map = document.getElementById("table-map");
    
    map.rows[x-1].cells[y-1].style.backgroundColor = "blue";
    map.rows[x-1].cells[y].style.backgroundColor = "blue";
    map.rows[x-1].cells[y+1].style.backgroundColor = "blue";

    map.rows[x].cells[y-1].style.backgroundColor = "blue";
    map.rows[x].cells[y].style.backgroundColor = "blue";
    map.rows[x].cells[y+1].style.backgroundColor = "blue";

    map.rows[x+1].cells[y-1].style.backgroundColor = "blue";
    map.rows[x+1].cells[y].style.backgroundColor = "blue";
    map.rows[x+1].cells[y+1].style.backgroundColor = "blue";

    robotPos = [20-x,y,null];

    console.log(robotPos);

    stepTwo();
}

function stepTwo() {
    var map = document.getElementById("table-map");
    document.getElementById("setup-instruct").innerHTML = "Step 2: Select robot direction.";

    document.getElementById("button-north").addEventListener("click", function() {
        map.rows[20-robotPos[0]-1].cells[robotPos[1]].style.backgroundColor = "#AAAAFF";
        robotPos = [robotPos[0],robotPos[1],"N"];
        clearRDirectionListeners();
    })
    document.getElementById("button-east").addEventListener("click", function() {
        map.rows[20-robotPos[0]].cells[robotPos[1]+1].style.backgroundColor = "#AAAAFF";
        robotPos = [robotPos[0],robotPos[1],"E"];
        clearRDirectionListeners();
    })
    document.getElementById("button-south").addEventListener("click", function() {
        map.rows[20-robotPos[0]+1].cells[robotPos[1]].style.backgroundColor = "#AAAAFF";
        robotPos = [robotPos[0],robotPos[1],"S"];
        clearRDirectionListeners();
    })
    document.getElementById("button-west").addEventListener("click", function() {
        map.rows[20-robotPos[0]].cells[robotPos[1]-1].style.backgroundColor = "#AAAAFF";
        robotPos = [robotPos[0],robotPos[1],"W"];
        clearRDirectionListeners();
    })
}

function clearRDirectionListeners() {
    var directions = document.getElementById("div-directions");
    var newDir = directions.cloneNode(true);
    directions.parentNode.replaceChild(newDir, directions);

    console.log(robotPos);

    stepThree();
}

function stepThree() {
    var map = document.getElementById("table-map");
    document.getElementById("setup-instruct").innerHTML =  `Step ${stepCount}: Select obstacle position.`;
    for (var i = 0, row; row = map.rows[i]; i++) {
        if (i < 20) {
            for (var j = 1, col; col = row.cells[j]; j++) {
                if (occupied(i,j)){
                    continue;
                }
                (function(x,y) {
                    col.addEventListener("mouseenter", () => tryObs(x,y));
                })(i,j);
                (function(x,y) {
                    col.addEventListener("mouseleave", () => unTryObs(x,y));
                })(i,j);
                (function(x,y) {
                    col.addEventListener("click", () => clearObsListeners(x,y));
                })(i,j);
            }
        }
    }
}

function occupied(x,y) {
    // robot pos
    if (x >= 20-robotPos[0]-1 & x <= 20-robotPos[0]+1) {
        if (y >= robotPos[1]-1 & y <= robotPos[1]+1) {
            return true;
        }
    }

    // obs pos
    if (obs.length > 0) {
        for (var i = 0; i < obs.length; i++) {
            if (x == 20-obs[i][0] & y == obs[i][1]) {
                return true;
            }
        }
    }
    
    return false;
}

function tryObs(x,y) {
    var map = document.getElementById("table-map");
    map.rows[x].cells[y].style.backgroundColor = "#FF000050";
}

function unTryObs(x,y) {
    var map = document.getElementById("table-map");
    map.rows[x].cells[y].style.backgroundColor = "";
}

function clearObsListeners(x,y) {
    var map = document.getElementById("table-map");
    var cloneMap = map.cloneNode(true);
    map.parentNode.replaceChild(cloneMap, map);

    map = document.getElementById("table-map");
    map.rows[x].cells[y].style.backgroundColor = "#FF0000";

    obs.push([20-x,y,null,obs.length+1]);
    console.log(obs);

    stepCount++;

    stepFour();
}

function stepFour() {
    var map = document.getElementById("table-map");
    document.getElementById("setup-instruct").innerHTML =  `Step ${stepCount}: Select obstacle direction.`;
    var curObsPos = obs[obs.length - 1];
    var curObs = map.rows[20-curObsPos[0]].cells[curObsPos[1]];

    document.getElementById("button-north").addEventListener("click", function() {
        curObs.style.borderTopWidth = "5px";
        obs[obs.length - 1] = [curObsPos[0],curObsPos[1],"N",curObsPos[3]];
        clearDDirectionListeners();
    })
    document.getElementById("button-east").addEventListener("click", function() {
        curObs.style.borderRightWidth = "5px";
        obs[obs.length - 1] = [curObsPos[0],curObsPos[1],"E",curObsPos[3]];
        clearDDirectionListeners();
    })
    document.getElementById("button-south").addEventListener("click", function() {
        curObs.style.borderBottomWidth = "5px";
        obs[obs.length - 1] = [curObsPos[0],curObsPos[1],"S",curObsPos[3]];
        clearDDirectionListeners();
    })
    document.getElementById("button-west").addEventListener("click", function() {
        curObs.style.borderLeftWidth = "5px";
        obs[obs.length - 1] = [curObsPos[0],curObsPos[1],"W",curObsPos[3]];
        clearDDirectionListeners();
    })
}

function clearDDirectionListeners() {
    var directions = document.getElementById("div-directions");
    var newDir = directions.cloneNode(true);
    directions.parentNode.replaceChild(newDir, directions);

    stepCount++;

    if (obs.length == 4) {
        document.getElementById("div-start").style.display = "block";
        document.getElementById("button-start").style.cursor = "pointer";
        document.getElementById("button-start").onclick = function() {
            sendSetup(robotPos, obs);
        }
    }

    if (obs.length < 8) {
        stepThree();
    } else {
        stepLast();
    }
}

function stepLast() {
    var map = document.getElementById("table-map");
    document.getElementById("setup-instruct").innerHTML =  `Step ${stepCount}: Press 'START'.`;
    var directionsDiv = document.getElementById("div-directions");
    directionsDiv.parentElement.removeChild(directionsDiv);
}