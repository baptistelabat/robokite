console.log("This is a kite simulator");

yaw =0
greatRoll=0 // Angle of rotation around wind axis 0 at zenith, pi/2 on the right for a kiter
greatPitch =0 // Angle between kite line and plan orthogonal to wind
greatRollRate = 0 // Derivative of great roll
greatPitchRate =0 // Derivative of great pitch
va = 10 // Apparent wind speed
v0=10 // wind speed
L=10 // Line length
glideRatio =5

steering = 0;
brake = 0;
reelOutSpeed = 0;

told=0
sampleTime = 0.01;


glideRatio0=5
brake2glideRatio = glideRatio0/100.0*0.9
steeringSquared2glideRatio= glideRatio0/(100.*100.*Math.PI*Math.PI/180./180.)/24

g =0.04*2 // Steering efficiency

function updateInput(){
	brake = params.brake;
	steering = params.steering*Math.PI/180.;
	reelOutSpeed = params.reelOutSpeed;
}

function updateGlideRatio(){
	// glideRatio is related to steering according to "Real-Time Optimizing Control of an experimental crosswind Power Kite"
	glideRatio = glideRatio0-steeringSquared2glideRatio*steering*steering-brake2glideRatio*brake;
}
function updateDerivative(){
	v0 = params.windspeed;
	L= params.lineLength;
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
	params.lineLength = L;
	params.greatRoll = greatRoll*180/Math.PI;
	params.greatPitch = greatPitch*180/Math.PI;
}

setInterval(updaten, 1);
var d = new Date();
var t0 = d.getTime();
simulation_time = 0;

function updaten()
{
  for (i=0;i<8;i++) // Dirty manual tuning to get close of real time on my computer
  {
    updateSimu();
  }
}

function updateSimu(){
  
  // Try to use real time
  d = new Date();
  t = d.getTime()-t0;
  dt = (t-told)/1000;
  told = t;
  //console.log(dt/sampleTime)
  // Use constant sampleTime instead (to avoid Nan for unknown reason)
  //dt = sampleTime// +0*dt;

  simulation_time = simulation_time + dt;
  
  updateInput();
  updateGlideRatio();
  updateDerivative()
  updateOut()
  eulerIntegration()
  //console.log(yaw);
  
}



function modm180p180(angle){
	angle =((angle +180 % 360 ) + 360 ) % 360 -180
	return angle;
}


  
  
