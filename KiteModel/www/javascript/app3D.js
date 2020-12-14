console.log("This is a kite simulator");

yaw =0
greatRoll=0 // Angle of rotation around wind axis 0 at zenith, pi/2 on the right for a kiter
greatPitch =0 // Angle between kite line and plan orthogonal to wind
greatRollRate = 0 // Derivative of great roll
greatPitchRate =0 // Derivative of great pitch
va = 10 // Apparent wind speed
v0=10 // wind speed
L=50 // Line length
glideRatio =5

steering =0 // Steering command
brake = 0; // Brake command
reelOutSpeed =0 // Clear enough
told=0
sampleTime = 0.01;
meter2pix = 5;


glideRatio0=5
brake2glideRatio = glideRatio0/100.0
steeringSquared2glideRatio= glideRatio0/(100.*100.*Math.PI*Math.PI/180./180.)/2

g =0.04 // Steering efficiency

function updateGlideRatio(){
	// glideRatio is related to steering according to "Real-Time Optimizing Control of an experimental crosswind Power Kite"
	glideRatio = glideRatio0-steeringSquared2glideRatio*steering*steering-brake2glideRatio*brake;
}
function updateDerivative(){
	yawRate=g*va*steering-greatRollRate*Math.sin(greatPitch)//+c2/va*Math.sin(yawg)*sin(inclination)
	va = v0*glideRatio*Math.sin(greatPitch)-reelOutSpeed*glideRatio
	greatPitchRate = -v0/L*(glideRatio*Math.cos(yaw)*Math.sin(greatPitch) -Math.cos(greatPitch))+reelOutSpeed/L*glideRatio*Math.cos(yaw)
	greatRollRate = (v0*glideRatio*Math.sin(greatPitch)-reelOutSpeed*glideRatio)/(L*Math.cos(greatPitch))*Math.sin(yaw)
}

function updateOut(){
		
		// Formula according to "A quaternion based model for optimal control of Airborne Wind Energy"
		F = Math.sqrt(glideRatio/Math.sqrt(1+Math.pow(glideRatio,2)))*va
}
function eulerIntegration(){
	L = L + reelOutSpeed*dt
	yaw = yaw + yawRate*dt
	greatRoll = greatRoll + greatRollRate*dt
	greatPitch = greatPitch + greatPitchRate*dt
}


document.getElementById("steeringRange").addEventListener("change", updateSteering);
document.getElementById("brakeRange").addEventListener("change", updateBrake);
document.getElementById("reelOutSpeedRange").addEventListener("change", updateReelOutSpeed);
document.getElementById("fluidVelocityRange").addEventListener("change", updateFluidVelocity);

setInterval(updaten, 1);
var d = new Date();
var t0 = d.getTime();
simulation_time = 0;

function updaten()
{
  for (i=0;i<8;i++) // Dirty manual tuning to get close of real time on my computer
  {
    update();
  }
  updateOutput()
}

function update(){
  
  // Try to use real time
  d = new Date();
  t = d.getTime()-t0;
  dt = (t-told)/1000;
  told = t;
  //console.log(dt/sampleTime)
  // Use constant sampleTime instead (to avoid Nan for unknown reason)
  //dt = sampleTime// +0*dt;

  simulation_time = simulation_time + dt;
  
  updateGlideRatio();
  updateDerivative()
  updateOut()
  eulerIntegration()
  plot();
  
}
function updateSteering(){
		//get elements
		var myRange = document.getElementById("steeringRange");
		var myOutput = document.getElementById("steering");
		//copy the value over
		myOutput.value = myRange.value;
		steering = myOutput.value*Math.PI/180;
	}
	function updateBrake(){
		//get elements
		var myRange = document.getElementById("brakeRange");
		var myOutput = document.getElementById("brake");
		//copy the value over
		myOutput.value = myRange.value;
		brake = myOutput.value;
	}
function updateReelOutSpeed(){
		//get elements
		var myRange = document.getElementById("reelOutSpeedRange");
		var myOutput = document.getElementById("reelOutSpeed");
		//copy the value over
		myOutput.value = myRange.value;
		reelOutSpeed = myOutput.value;
}
function updateFluidVelocity(){
		//get elements
		var myRange = document.getElementById("fluidVelocityRange");
		var myOutput = document.getElementById("fluidVelocity");
		//copy the value over
		myOutput.value = myRange.value;
		average_fluid_speed = 1*myOutput.value;
}

function modm180p180(angle){
	angle =((angle +180 % 360 ) + 360 ) % 360 -180
	return angle;
}
function updateOutput(){
	myOutput = document.getElementById("time");
    myOutput.value = Math.round(simulation_time*100)/100;
    var myOutput = document.getElementById("lineLength");
    myOutput.value = Math.round(L*10)/10 ;
    var myOutput = document.getElementById("yaw_deg");
    myOutput.value = Math.round(modm180p180(yaw*180/Math.PI)*10)/10 ;
    
        var myOutput = document.getElementById("greatRoll_deg");
    myOutput.value = Math.round(modm180p180(greatRoll*180/Math.PI)*10)/10;
    
        var myOutput = document.getElementById("greatPitch_deg");
    myOutput.value = Math.round(greatPitch*180/Math.PI*10)/10;
    
        
    myOutput = document.getElementById("yawRate_degps");
    myOutput.value = Math.round(yawRate*180/Math.PI*10)/10;
    

    
    myOutput = document.getElementById("greatRollRate_degps");
    myOutput.value = Math.round(greatRollRate*180/Math.PI*10)/10;
    
    myOutput = document.getElementById("greatPitchRate_degps");
    myOutput.value = Math.round(greatPitchRate*180/Math.PI*10)/100;
}
function rotateKite(r){
    kite = document.getElementById("kite_frame");
        r_deg = greatRoll*180/Math.PI+yaw*180/Math.PI;
		kite.setAttribute('transform', 'rotate(' +r_deg +')');
		}
function plotKite(){
  span = 5;
  kite = document.getElementById("kite");
  kite.setAttribute('x1', -1/2.*span*meter2pix);
  kite.setAttribute('x2',  1/2.*span*meter2pix);
}
function translateKite(y, z){
  kite_frame = document.getElementById("local_frame");
  kite_line = document.getElementById("kite_line");
  kite_frame.setAttribute('transform', 'translate(' +y*meter2pix +','+ -z*meter2pix +')');
  kite_line.setAttribute('x2', y*meter2pix);
  kite_line.setAttribute('y2', -z*meter2pix);
}
function plot(){
  plotKite();
  rotateKite();
  translateKite(L*Math.sin(greatRoll)*Math.cos(greatPitch), L*Math.cos(greatRoll)*Math.cos(greatPitch));
}

  
  
