var ws = new WebSocket("ws://localhost:8080/websocket");
ws.onmessage = function(evt){
	myOutput.value = evt.data;
}

var inputs = document.getElementsByTagName('input');
for(var i = 0; i < inputs.length; i++) {
	if(inputs[i].type == 'range') {
		inputs[i].addEventListener('click', function() {
			this.focus(); 
		});
	}
}

function sendOrder1(){
  var msg = {
    type: "message",
    value:  document.getElementById("setpoint1Number").value,
    id:   "pwm1",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}

function sendOrder2(){
  var msg = {
    type: "message",
    value:  document.getElementById("setpoint2Number").value,
    id:   "pwm2",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKp(){
  var msg = {
    type: "message",
    value:  document.getElementById("KpNumber").value,
    id:   "kp",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKi(){
  var msg = {
    type: "message",
    value:  document.getElementById("KiNumber").value,
    id:   "ki",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKd(){
  var msg = {
    type: "message",
    value:  document.getElementById("KdNumber").value,
    id:   "kd",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}

function sendControlMode(){
  var msg = {
    type: "message",
    value:  document.getElementById("controlMode").value,
    id:   "controlMode",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}

function sendSetpointOrigin(){
  var msg = {
    type: "message",
    value:  document.getElementById("setpointOrigin").value,
    id:   "setPointOrigin",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}

function sendControlVariable(){
  var msg = {
    type: "message",
    value:  document.getElementById("controlVariable").value,
    id:   "controlVariable",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}

function setControlMode(){
	var radios = document.getElementsByName('controlModeRequest');

for (var i = 0, length = radios.length; i < length; i++) {
    if (radios[i].checked) {
        // do whatever you want with the checked radio

        // only one radio can be logically checked, don't check the rest
        break;
    }
}
var myOutput = document.getElementById("controlMode");
myOutput.value = radios[i].value;
sendControlMode();
} // end change

function setSetpointOrigin(){
	var radios = document.getElementsByName('setpointOriginRequest');

for (var i = 0, length = radios.length; i < length; i++) {
    if (radios[i].checked) {
        // do whatever you want with the checked radio

        // only one radio can be logically checked, don't check the rest
        break;
    }
}

var myOutput = document.getElementById("setpointOrigin");
myOutput.value = radios[i].value;
sendSetpointOrigin();
} // end change


function setControlVariable(){
	var radios = document.getElementsByName('controlVariableRequest');

for (var i = 0, length = radios.length; i < length; i++) {
    if (radios[i].checked) {
        // do whatever you want with the checked radio

        // only one radio can be logically checked, don't check the rest
        break;
    }
}
var myOutput = document.getElementById("controlVariable");
myOutput.value = radios[i].value;
sendControlVariable();
} // end change

function updateSetpoint1Range(){
//get elements
var myRange = document.getElementById("setpoint1Range");
var myNumber = document.getElementById("setpoint1Number");
//copy the value over
myNumber.value = myRange.value;
sendOrder1();
} // end function

function updateSetpoint1Number(){
//get elements
var myRange = document.getElementById("setpoint1Range");
var myNumber = document.getElementById("setpoint1Number");
//copy the value over
myRange.value = myNumber.value;
sendOrder1();
} // end function

function updateSetpoint2Range(){
//get elements
var myRange = document.getElementById("setpoint2Range");
var myNumber = document.getElementById("setpoint2Number");
//copy the value over
myNumber.value = myRange.value;
sendOrder2();
} // end function

function updateSetpoint2Number(){
//get elements
var myRange = document.getElementById("setpoint2Range");
var myNumber = document.getElementById("setpoint2Number");
//copy the value over
myRange.value = myNumber.value;
sendOrder2();
} // end function

function updateKpRange(){
//get elements
var myRange = document.getElementById("KpRange");
var myNumber = document.getElementById("KpNumber");
//copy the value over
myNumber.value = myRange.value;
sendKp();
} // end function

function updateKpNumber(){
//get elements
var myRange = document.getElementById("KpRange");
var myNumber = document.getElementById("KpNumber");
//copy the value over
myRange.value = myNumber.value;
sendKp();
} // end function

function updateKiRange(){
//get elements
var myRange = document.getElementById("KiRange");
var myNumber = document.getElementById("KiNumber");
//copy the value over
myNumber.value = myRange.value;
sendKi();
} // end function

function updateKiNumber(){
//get elements
var myRange = document.getElementById("KiRange");
var myNumber = document.getElementById("KiNumber");
//copy the value over
myRange.value = myNumber.value;
sendKi();
} // end function

function updateKdRange(){
//get elements
var myRange = document.getElementById("KdRange");
var myNumber = document.getElementById("KdNumber");
//copy the value over
myNumber.value = myRange.value;
sendKd();
} // end function

function updateKdNumber(){
//get elements
var myRange = document.getElementById("KdRange");
var myNumber = document.getElementById("KdNumber");
//copy the value over
myRange.value = myNumber.value;
sendKd();
} // end function


