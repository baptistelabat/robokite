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
function updateOutput(){
//get elements
var myRange = document.getElementById("myRange");
var myOutput = document.getElementById("myOutput2");
//copy the value over
myOutput.value = myRange.value;
} // end function
