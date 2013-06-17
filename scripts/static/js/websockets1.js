$(document).ready(function() {

    if ("WebSocket" in window) {
	var loc = window.location, new_uri;
	if (loc.protocol === "https:") {
	    new_uri = "wss:";
	} else {
	    new_uri = "ws:";
	}
	new_uri += "//" + loc.host;
	new_uri += loc.pathname + "/api";
	ws = new WebSocket(new_uri);
	ws.onmessage = function(msg) {
	    $("#log").append("<p>" + msg.data + "</p>")
	};
    } else {
	alert("WebSocket not supported");
    }
});

//Depracated
//Sending methods for directions.
function goTop() {
    ws.send("top")
    return true;
}

function goBack() {
    ws.send("bottom")
    return true;
}

function goRight() {
    ws.send("right");
    return true;
}

function goLeft() {
    ws.send("left");
    return true;
}

function stopMove() {
    ws.send("s");
    return true;
}

//Methode d'envoie d'une position de destination
function goThere(x, y) {
    var x_string = x.toString();
    var y_string = y.toString();
    ws.send("pos_" + x_string + "_" + y_string);
    return true;
}

//Methode d'envoie d'un ordre du joystick
function sendOrder(speed, turn) {
    var speed_str = speed.toString();
    var turn_str = turn.toString();
    ws.send("st_" + speed_str + "_" + turn_str);
    return true;
}

//Methode d'envoie d'une commande de parole
function say(str) {
    ws.send(str);
    return true;
}

//Methode d'annulation d'une position d'arrivee
function cancel() {
    ws.send("cancel");
    return true;
}

