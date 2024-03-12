var xhr = null;
var pathArray = null;

getXmlHttpRequestObject = function() {
    if (!xhr) {
        xhr = new XMLHttpRequest();
    }
    return xhr;
}

function receiveInstructions() {
    if (xhr.readyState == 4 && xhr.status == 200) {
        res = JSON.parse(xhr.responseText);
        pathArray = res["path"];
        console.log(pathArray);
        pathSetup();
    }
}

function sendSetup(robotPos, obs) {
    var map = document.getElementById("table-map");
    var cloneMap = map.cloneNode(true);
    map.parentNode.replaceChild(cloneMap, map);
    var startDiv = document.getElementById("div-start");
    startDiv.parentElement.removeChild(startDiv);
    xhr = getXmlHttpRequestObject();
    xhr.onreadystatechange = receiveInstructions;
    xhr.open("POST","http://localhost:5000/get-path", true);
    xhr.setRequestHeader("Content-type","application/json");
    xhr.send(JSON.stringify({
        "robotPos": robotPos,
        "obs": obs
    }));
}