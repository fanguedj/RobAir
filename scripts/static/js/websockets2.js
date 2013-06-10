$(document).ready(function() {
	alert("Test");
	if ("WebSocket" in window) {
		alert("WS");
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
	ws.send($('#top').val())
	alert("top");
	return true;
}

function goBack() {
	ws.send($('#back').val())
	return true;
}

function goRight() {
	ws.send("right");
//	return true;
}

function goLeft() {
	ws.send("left");
//	return true;
}
