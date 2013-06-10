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
