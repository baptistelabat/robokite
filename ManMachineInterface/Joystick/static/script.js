function updateJoyRange(){
//get elements
var myRange = document.getElementById("JoyRange");
var myNumber = document.getElementById("JoyNumber");
//copy the value over
myNumber.value = myRange.value;
} // end function

function updateJoyNumber(){
//get elements
var myRange = document.getElementById("JoyRange");
var myNumber = document.getElementById("JoyNumber");
//copy the value over
myRange.value = myNumber.value;
} // end function

function updateJoyHorizonRange(){
//get elements
var myRange = document.getElementById("JoyHorizonRange");
var myNumber = document.getElementById("JoyHorizonNumber");
//copy the value over
myNumber.value = myRange.value;
} // end function

function updateJoyHorizonNumber(){
//get elements
var myRange = document.getElementById("JoyHorizonRange");
var myNumber = document.getElementById("JoyHorizonNumber");
//copy the value over
myRange.value = myNumber.value;
} // end function

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
        var myRange = document.getElementById("JoyRange");
		var myNumber = document.getElementById("JoyNumber");
		y = 100*pad.axes[3];
		if (previous_y != y){
			myRange.value = y;
			myNumber.value = y;
		}
		var myRange = document.getElementById("JoyHorizonRange");
		var myNumber = document.getElementById("JoyHorizonNumber");
		x = 100*pad.axes[2];
		if (previous_x != x){
			myRange.value = x;
			myNumber.value = x;
		}
		previous_x = x;
		previous_y = y;
    }
}

var x, y;
var previous_x, previous_y;
window.requestAnimationFrame(runAnimation);
