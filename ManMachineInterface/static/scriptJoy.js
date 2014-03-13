function runAnimation()
{
    window.requestAnimationFrame(runAnimation);

    var gamepads = (navigator.webkitGetGamepads && navigator.webkitGetGamepads()) ||
          !!navigator.webkitGamepads || !!navigator.mozGamepads ||
          !!navigator.msGamepads || !!navigator.gamepads;

    for (var i = 0; i < gamepads.length; ++i)
    {
        var pad = gamepads[i];
        console.log(pad.axes);
        var myRange = document.getElementById("setpoint1Range");
		var myNumber = document.getElementById("setpoint1Number");
		y = 100*pad.axes[3];
		if (previous_y != y){
			myRange.value = y;
			myNumber.value = y;
			sendOrder1()
		}
		var myRange = document.getElementById("setpoint2Range");
		var myNumber = document.getElementById("setpoint2Number");
		x = 100*pad.axes[2];
		if (previous_x != x){
			myRange.value = x;
			myNumber.value = x;
			sendOrder2()
		}
		previous_x = x;
		previous_y = y;
    }
}

var x, y;
var previous_x, previous_y;
window.requestAnimationFrame(runAnimation);
