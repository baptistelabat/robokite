function DispatchText(){
                var userInput = document.getElementById("setOrientationRange").value;
                ws.send(userInput);
}
function setManualControl(){
var myOutput = document.getElementById("controlMode");
myOutput.value = "Manual control";
} // end change

function setJoystickControl(){
var myOutput = document.getElementById("controlMode");
myOutput.value = "Joystick control";
} // end change

function setAutomaticControl(){
var myOutput = document.getElementById("controlMode");
myOutput.value = "Automatic control";
} // end change

function updateSetOrientation(){
//get elements
var myRange = document.getElementById("setOrientationRange");
var myOutput = document.getElementById("setOrientation");
//copy the value over
myOutput.value = myRange.value;
DispatchText()
} // end function

function updateKp(){
//get elements
var myRange = document.getElementById("KpRange");
var myOutput = document.getElementById("Kp");
//copy the value over
myOutput.value = myRange.value;
} // end function

function updateKi(){
//get elements
var myRange = document.getElementById("KiRange");
var myOutput = document.getElementById("Ki");
//copy the value over
myOutput.value = myRange.value;
} // end function

function updateKd(){
//get elements
var myRange = document.getElementById("KdRange");
var myOutput = document.getElementById("Kd");
//copy the value over
myOutput.value = myRange.value;
} // end function


