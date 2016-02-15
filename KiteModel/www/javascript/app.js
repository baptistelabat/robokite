console.log("This is a kite simulator");
var CL = [];
var CD = [];
var alpha_deg = [];
function liftCoefficient(alpha){
  // This is a simplified formula for lift coefficient
  // Fit with lift for an infinite elliptic wing for small angle
  // Maximum lift at 45°
  // No lift at 90°
  // Negative lift from 90°
  Cl = Math.PI*Math.sin (2*alpha);
  
  if (CL.length >0)
  {
    Cl = everpolate.linear(alpha*180/Math.PI, alpha_deg, CL);
  }

  return Cl;
 }
function dragCoefficient(alpha){
  Cd0 = 0.1;
  AR = 5; // Aspect ratio
  e = 1;
  inducedDragCoefficient = Math.pow(2*Math.PI*Math.sin(alpha),2)/(Math.PI*AR*e);
  Cd = inducedDragCoefficient + Cd0;
  if (CD.length >0)
  {
    Cd = everpolate.linear(alpha*180/Math.PI, alpha_deg, CD);
  }
  return Cd;
}
function wind_profile(w10, z){
  // Compute the effective wind at this altitude using log wind profil
  // http://en.wikipedia.org/wiki/Log_wind_profile
  Zref = 10.  ; // Reference altitude for wind measurements (m)
  Zo = 0.055    // longueur de rugosite du terrain (m)
  
  // Saturate z in order not to fall to negative wind or zero wind
  z = Math.max(z, 10*Zo)
  return w10*Math.log(z/Zo)/Math.log(Zref/Zo) //Log wind profil
}
var V = 10;

line_length     = 10;
wind_velocity   = 10;
kite_mass       = 2;  
kite_surface    = 6;  
rho_air         = 1;    // Air density
elevation0      = 0;
omega0          = 0;    // Angular rate
AoKdeg          = 50;   // Angle of Keying (calage)
sampleTime      = 0.0005; // Sample time
earth_gravity              = 9.81;
g= earth_gravity;
meter2pix = 20;
reel_speed = 0;

AoK = AoKdeg*Math.PI/180;
omega = 0;
elevation = 0;
y_base = 0;
z_base = 0;


// y is horizontal and positive in wind propagation direction
// z is vertical and positive up

// Base velocity relative to ground projected in ground axis
// Base is assumed to be static
v_base = 0;
w_base = 0;
document.getElementById("angleOfKeyRange").addEventListener("change", updateAngleOfKey);
document.getElementById("lineLengthRange").addEventListener("change", updateLineLength);
document.getElementById("windVelocityRange").addEventListener("change", updateWindVelocity);
document.getElementById("kiteMassRange").addEventListener("change", updateKiteMass);
document.getElementById("kiteSurfaceRange").addEventListener("change", updateKiteSurface);
document.getElementById("myCheck").addEventListener("change", updateGravity);
document.getElementById("reelSpeedRange").addEventListener("change", updateReelSpeed);
setInterval(updaten, 1);
setInterval(updatePlot,100);
var d = new Date();
var t0 = d.getTime();
told = 0;
simulation_time = 0;

function plot(y_base, z_base, y_kite, z_kite, pitch){
  rotateKite(pitch);
  translateKite(y_kite, z_kite);
}
function updatePlot(){
  plot(y_base, z_base, y_kite, z_kite, pitch);
  updateOutput();
  setLineLength();
}
function updaten()
{
  for (i=0;i<8;i++) // Dirty manual tuning to get close of real time on my computer
  {
    update();
  }
}

//function update(dt, AoK){
function update(){
  
  // Try to use real time
  d = new Date();
  t = d.getTime()-t0;
  dt = (t-told)/1000;
  told = t;
  //console.log(dt/sampleTime)
  // Use constant sampleTime instead (to avoid Nan for unknown reason)
  dt = sampleTime// +0*dt;

  // Compute kite position
  y_kite = y_base + line_length * Math.cos(elevation);
  z_kite = z_base + line_length * Math.sin(elevation);
  
  pitch = AoK -elevation;
  //console.log(pitch);

  // Kite velocity relative to ground, projected in ground axis
  // Line is assumed straight.
  v_kite = v_base - omega*line_length*Math.cos(Math.PI/2 - elevation) + reel_speed*Math.cos(elevation);
  w_kite = w_base + omega*line_length*Math.sin(Math.PI/2 - elevation) + reel_speed*Math.sin(elevation);

  // Wind velocity: air velocity relative to ground, projected in ground axis
  // Assumed to be constant in time and space and horizontal
  v_wind = wind_profile(wind_velocity, z_kite);
  w_wind = 0;

  // Wind relative velocity : air velocity relative to kite, projected in ground axis
  v_air_kite = v_wind-v_kite;
  w_air_kite = w_wind-w_kite;

  // Angle of attack of the kite, defined between kite chord and relative air velocity
  angle_air_kite = Math.atan2(w_air_kite, v_air_kite);
  AoA = angle_air_kite +pitch;
  //console.log(AoA);
  // Dynamic pressure
  q = 1/2*rho_air *(v_air_kite*v_air_kite + w_air_kite*w_air_kite);

  // Lift and drag are in apparent wind frame
  lift   = q*kite_surface*liftCoefficient(AoA);
  drag   = q*kite_surface*dragCoefficient(AoA);
  
  // Rotate to ground frame
  Fz = lift* Math.cos(angle_air_kite) + drag*Math.sin(angle_air_kite);
  Fy = -lift* Math.sin(angle_air_kite) + drag*Math.cos(angle_air_kite);

  // Torque computed at base
  ML = +Fz * y_kite-kite_mass*g*y_kite;
  MD = -Fy * z_kite;
  

  // Angular acceleration
  omegap = 1/(kite_mass*line_length^2) * (ML + MD)- 0.0*omega;  //x*omega = amortissement
  //console.log(omegap);
  // Saturate to avoid instabilities
  omegap = Math.max(-60000, Math.min(omegap,60000));

  simulation_time = simulation_time + dt;
  
  // Compute line length
  line_length = line_length + reel_speed*dt;
  if (line_length<2)
  {
    reel_speed = 0;
    line_length = 2;
  }
  
  omega = omega + omegap * dt;
  
  // Saturate to avoid divergences 
  //console.log(omega);
  omega = Math.max(-60, Math.min(omega,60));
  
  elevation = elevation + omega * dt;
  y_base = y_base + v_base*dt;
  z_base = z_base + w_base*dt;
  
    // Compute line tension
  line_tension = Fy*Math.cos(elevation) +  (Fz-kite_mass*g)*Math.sin(elevation) + kite_mass*omega*omega*line_length;
}
function rotateKite(r){
    kite = document.getElementById("kite");
        r_deg = r*180/Math.PI;
		kite.setAttribute('transform', 'rotate(' +r_deg +')');
		}
function translateKite(y, z){
  kite_frame = document.getElementById("local_frame");
  kite_line = document.getElementById("kite_line");
  kite_frame.setAttribute('transform', 'translate(' +y*meter2pix +','+ -z*meter2pix +')');
  kite_line.setAttribute('x2', y*meter2pix);
  kite_line.setAttribute('y2', -z*meter2pix);
}

function updateAngleOfKey(){
		//get elements
		var myRange = document.getElementById("angleOfKeyRange");
		var myOutput = document.getElementById("angleOfKey");
		//copy the value over
		myOutput.value = myRange.value;
    AoK = myOutput.value*Math.PI/180;
	}
function updateLineLength(){
		//get elements
		var myRange = document.getElementById("lineLengthRange");
		var myOutput = document.getElementById("lineLength");
		//copy the value over
		myOutput.value = myRange.value;
    //line_length = myOutput.value;
	}
function setLineLength(){
		//get elements
		var myRange = document.getElementById("lineLengthRange");
		var myOutput = document.getElementById("lineLength");
		//copy the value over
		myOutput.value = Math.round(line_length*10)/10;
    myRange.value = line_length;
	}
function updateReelSpeed(){
		//get elements
		var myRange = document.getElementById("reelSpeedRange");
		var myOutput = document.getElementById("reelSpeed");
		//copy the value over
		myOutput.value = myRange.value;
    reel_speed = myOutput.value;
	}
  function updateWindVelocity(){
		//get elements
		var myRange = document.getElementById("windVelocityRange");
		var myOutput = document.getElementById("windVelocity");
		//copy the value over
		myOutput.value = myRange.value;
    wind_velocity = myOutput.value;
	}
  function updateKiteMass(){
		//get elements
		var myRange = document.getElementById("kiteMassRange");
		var myOutput = document.getElementById("kiteMass");
		//copy the value over
		myOutput.value = myRange.value;
    kite_mass = myOutput.value;
	}
  function updateKiteSurface(){
		//get elements
		var myRange = document.getElementById("kiteSurfaceRange");
		var myOutput = document.getElementById("kiteSurface");
		//copy the value over
		myOutput.value = myRange.value;
    kite_surface = myOutput.value;
	}
  function updateGravity(){
		//get elements
    
		var myCheck = document.getElementById("myCheck");
    g = earth_gravity*myCheck.checked;
	}
  function updateOutput(){
    var myOutput = document.getElementById("elevation");
    myOutput.value = Math.round(elevation*180/Math.PI*10)/10;
    
    myOutput = document.getElementById("time");
    myOutput.value = Math.round(simulation_time*100)/100;
    
    myOutput = document.getElementById("lineTension");
    myOutput.value = Math.round(line_tension*10)/100;
    
    myOutput = document.getElementById("kiteSpeed");
    myOutput.value = Math.round(omega*line_length*10)/10;
  }

