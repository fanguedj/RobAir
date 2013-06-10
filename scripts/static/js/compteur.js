$(function() {
	// possibilit� de donner une date pour la limite et ne rien changer en dessous
	// si la date est d�pass�e comme ici vous pouvez r�gler en nb de jours en dessous
	var note = $('#note'), ts = new Date(2012, 0, 1), newYear = true;

	if ((new Date()) > ts) {
		// dans ce cas 10*24*60*60*1000 repr�sente 10j 24h 60' 60"
		// pour 2h on �crirait 1*2*60*60*1000
		// IMPORTANT : ne pas supprimer le *1000
		ts = (new Date()).getTime() + 1 * 2 * 60 * 60 * 1000;
		newYear = false;
	}

	$('#countdown').countdown({
		timestamp : ts,
		callback : function(days, hours, minutes, seconds) {

			var message = "";
			message += hours + " heure" + (hours == 1 ? '' : 's' ) + ", ";
			message += minutes + " minute" + (minutes == 1 ? '' : 's' ) + " and ";
			message += seconds + " seconde" + (seconds == 1 ? '' : 's' ) + " <br />";

			if (newYear) {
				message += "jusqu'� la fin";
				// ou indiquez la date
			} else {
				message += ".";
			}

			note.html(message);
		}
	});

}); 