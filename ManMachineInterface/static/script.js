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
function sendKp1(){
  var msg = {
    type: "message",
    value:  document.getElementById("Kp1Number").value,
    id:   "kp1",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKi1(){
  var msg = {
    type: "message",
    value:  document.getElementById("Ki1Number").value,
    id:   "ki1",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKd1(){
  var msg = {
    type: "message",
    value:  document.getElementById("Kd1Number").value,
    id:   "kd1",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKp2(){
  var msg = {
    type: "message",
    value:  document.getElementById("Kp2Number").value,
    id:   "kp2",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKi2(){
  var msg = {
    type: "message",
    value:  document.getElementById("Ki2Number").value,
    id:   "ki2",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}

function sendKd2(){
  var msg = {
    type: "message",
    value:  document.getElementById("Kd2Number").value,
    id:   "kd2",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}

function sendKpRoll(){
  var msg = {
    type: "message",
    value:  document.getElementById("KpRollNumber").value,
    id:   "kpRoll",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKiRoll(){
  var msg = {
    type: "message",
    value:  document.getElementById("KiRollNumber").value,
    id:   "kiRoll",
    date: Date.now()
  };

  // Send the msg object as a JSON-formatted string.
  ws.send(JSON.stringify(msg));
}
function sendKdRoll(){
  var msg = {
    type: "message",
    value:  document.getElementById("KdRollNumber").value,
    id:   "kdRoll",
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

function updateKpRollRange(){
//get elements
var myRange = document.getElementById("KpRollRange");
var myNumber = document.getElementById("KpRollNumber");
//copy the value over
myNumber.value = myRange.value;
sendKpRoll();
} // end function

function updateKpRollNumber(){
//get elements
var myRange = document.getElementById("KpRollRange");
var myNumber = document.getElementById("KpRollNumber");
//copy the value over
myRange.value = myNumber.value;
sendKpRoll();
} // end function

function updateKiRollRange(){
//get elements
var myRange = document.getElementById("KiRollRange");
var myNumber = document.getElementById("KiRollNumber");
//copy the value over
myNumber.value = myRange.value;
sendKiRoll();
} // end function

function updateKiRollNumber(){
//get elements
var myRange = document.getElementById("KiRollRange");
var myNumber = document.getElementById("KiRollNumber");
//copy the value over
myRange.value = myNumber.value;
sendKiRoll();
} // end function

function updateKdRollRange(){
//get elements
var myRange = document.getElementById("KdRollRange");
var myNumber = document.getElementById("KdRollNumber");
//copy the value over
myNumber.value = myRange.value;
sendKdRoll();
} // end function

function updateKdRollNumber(){
//get elements
var myRange = document.getElementById("KdRollRange");
var myNumber = document.getElementById("KdRollNumber");
//copy the value over
myRange.value = myNumber.value;
sendKdRoll();
} // end function

function updateKp1Range(){
//get elements
var myRange = document.getElementById("Kp1Range");
var myNumber = document.getElementById("Kp1Number");
//copy the value over
myNumber.value = myRange.value;
sendKp1();
} // end function

function updateKp1Number(){
//get elements
var myRange = document.getElementById("Kp1Range");
var myNumber = document.getElementById("Kp1Number");
//copy the value over
myRange.value = myNumber.value;
sendKp1();
} // end function

function updateKi1Range(){
//get elements
var myRange = document.getElementById("Ki1Range");
var myNumber = document.getElementById("Ki1Number");
//copy the value over
myNumber.value = myRange.value;
sendKi1();
} // end function

function updateKi1Number(){
//get elements
var myRange = document.getElementById("Ki1Range");
var myNumber = document.getElementById("Ki1Number");
//copy the value over
myRange.value = myNumber.value;
sendKi1();
} // end function

function updateKd1Range(){
//get elements
var myRange = document.getElementById("Kd1Range");
var myNumber = document.getElementById("Kd1Number");
//copy the value over
myNumber.value = myRange.value;
sendKd1();
} // end function

function updateKd1Number(){
//get elements
var myRange = document.getElementById("Kd1Range");
var myNumber = document.getElementById("Kd1Number");
//copy the value over
myRange.value = myNumber.value;
sendKd1();
} // end function

function updateKp2Range(){
//get elements
var myRange = document.getElementById("Kp2Range");
var myNumber = document.getElementById("Kp2Number");
//copy the value over
myNumber.value = myRange.value;
sendKp2();
} // end function

function updateKp2Number(){
//get elements
var myRange = document.getElementById("Kp2Range");
var myNumber = document.getElementById("Kp2Number");
//copy the value over
myRange.value = myNumber.value;
sendKp2();
} // end function

function updateKi2Range(){
//get elements
var myRange = document.getElementById("Ki2Range");
var myNumber = document.getElementById("Ki2Number");
//copy the value over
myNumber.value = myRange.value;
sendKi2();
} // end function

function updateKi2Number(){
//get elements
var myRange = document.getElementById("Ki2Range");
var myNumber = document.getElementById("Ki2Number");
//copy the value over
myRange.value = myNumber.value;
sendKi2();
} // end function

function updateKd2Range(){
//get elements
var myRange = document.getElementById("Kd2Range");
var myNumber = document.getElementById("Kd2Number");
//copy the value over
myNumber.value = myRange.value;
sendKd2();
} // end function

function updateKd2Number(){
//get elements
var myRange = document.getElementById("Kd2Range");
var myNumber = document.getElementById("Kd2Number");
//copy the value over
myRange.value = myNumber.value;
sendKd2();
} // end function

ws.onmessage = function(evt){
	var myOutput = document.getElementById("serverFeedback");
	myOutput.value = evt.data;
}


