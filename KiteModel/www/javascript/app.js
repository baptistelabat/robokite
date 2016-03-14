console.log("This is a kite simulator");
var CL = [];
var CD = [];
var alpha_deg = [];
function liftCoefficient(alpha){
  // This is a simplified formula for lift coefficient
  // Maximum lift at 45°
  // No lift at 90°
  // Negative lift from 90°
  //http://people.clarkson.edu/~pmarzocc/AE429/AE-429-4.pdf
  dCl = 2*Math.PI; //infinite elliptic wing for small angle
  
  //AR= span^2/kite_surface;
  AR = 5;
  e  = 1// Oswald efficiency factor
  //alphai= Cl/(Math.PI*e*AR);
  
  //dCL = dCl*AR/(AR+2.5)
  dCL = dCl/(1+dCl/(Math.PI*e*AR))
  Cl = dCL/2.*Math.sin (2*alpha);
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
function wind_profile(w10, kite_position){
  // Compute the effective wind at this altitude using log wind profil
  // http://en.wikipedia.org/wiki/Log_wind_profile
  Zref = 10.  ; // Reference altitude for wind measurements (m)
  Zo = 0.055    // longueur de rugosite du terrain (m)
  
  // Saturate z in order not to fall to negative wind or zero wind
  z = Math.max(kite_position.z, 10*Zo)
  return w10//*Math.log(z/Zo)/Math.log(Zref/Zo) //Log wind profil
}
var V = 10;

line_length     = 10;
wind_speed      = 10;
kite_mass       = 2;  
kite_surface    = 6;  
rho_air         = 1;    // Air density
elevation0      = 0;
pqr0          = 0;    // Angular rate
AoKdeg          = 50;   // Angle of Keying (calage)
sampleTime      = 0.0005; // Sample time
earth_gravity              = 9.81;
g= earth_gravity;
meter2pix = 20;
reel_speed = 0;
pitch=0;
line_tension = 0;

AoK = AoKdeg*Math.PI/180;

elevation = 0;
kite_position = new THREE.Vector3( 0, 0, 0 );
kite_velocity = new THREE.Vector3( 0, 0, 0 );
base_position = new THREE.Vector3( 0, 0, 0 );
base_velocity = new THREE.Vector3( 0, 0, 0 );
wind_velocity = new THREE.Vector3( 0, 0, 0 );
Faero         = new THREE.Vector3( 0, 0, 0 );
Fline         = new THREE.Vector3( 0, 0, 0 );
Fweight       = new THREE.Vector3( 0, 0, 0 );
Fsum          = new THREE.Vector3( 0, 0, 0 );
torque_at_base = new THREE.Vector3( 0, 0, 0 );
pqr = new THREE.Vector3( 0, 0, 0 );
pqr.x = pqr0;

kite_position.set(0, line_length, 0); 

is1dof=true;

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

function plot(kite_position, pitch){
  rotateKite(pitch);
  translateKite(kite_position.y, kite_position.z);
}
function updatePlot(){
  plot(kite_position, pitch);
  updateOutput();
  if (reel_speed!=0)
  {
    setLineLength();
  }
}
function updaten()
{
  for (i=0;i<8;i++) // Dirty manual tuning to get close of real time on my computer
  {
    update();
  }
}

//function update(dt, AoK){
function computeForces(){

  pitch = AoK -elevation;
  //console.log(pitch);

  // Wind velocity: air velocity relative to ground, projected in ground axis
  // Assumed to be constant in time and space and horizontal
  wind_velocity.set(0, wind_profile(wind_speed, kite_position), 0);
  //console.log("Wind y z", wind_velocity.y, wind_velocity.z)

  // Wind relative velocity : air velocity relative to kite, projected in ground axis
  wind_relative_velocity = wind_velocity.sub(kite_velocity);
  //console.log("Kite velocity", kite_velocity.y, kite_velocity.z)
  //console.log("WindR y z", wind_relative_velocity.y, wind_relative_velocity.z)

  // Angle of attack of the kite, defined between kite chord and relative air velocity
  angle_air_kite = Math.atan2(wind_relative_velocity.z, wind_relative_velocity.y);
  AoA = angle_air_kite +pitch;
  //console.log(AoA);
  // Dynamic pressure
  v_air_kite = 0
  w_air_kite = 0
  q = 1/2*rho_air *Math.pow(wind_relative_velocity.length(),2);
  //console.log(q);
  // Lift and drag are in apparent wind frame
  lift   = q*kite_surface*liftCoefficient(AoA);
  drag   = q*kite_surface*dragCoefficient(AoA);
  //console.log(lift)
  
  // Rotate to ground frame
  CTM = new THREE.Matrix4;
  CTM.makeRotationX(angle_air_kite);
  //console.log(CTM);
  //console.log(CTM)
  Faero.set(  0,drag, lift);
  Faero.applyMatrix4(CTM);
  //console.log("Faero", Faero);
  Fweight.set(0, 0, -kite_mass*g);

  //console.log("Fweight", Fweight.x)
  extension = kite_position.sub(base_position).length()-line_length;
  //console.log("Extension", extension)
  YoungModulus = 34.5e3;// Kevlar
   if (is1dof)
   {
      YoungModulus=0;
   }
  line_diameter = 0.001;
  line_section =Math.PI*(line_diameter/2)^2;
  line_tension = extension/line_length *YoungModulus*line_section;
  line_tension=Math.max(0, line_tension);
  //console.log("Tension", line_tension)
  Fline.set(0, -line_tension * Math.cos(elevation), -line_tension * Math.sin(elevation));

  Fsum = Faero.add(Fline).add(Fweight);
  //console.log("Fsum",Fsum);
}
function update(){
  
  // Try to use real time
  d = new Date();
  t = d.getTime()-t0;
  dt = (t-told)/1000;
  told = t;
  //console.log(dt/sampleTime)
  // Use constant sampleTime instead (to avoid Nan for unknown reason)
  dt = sampleTime// +0*dt;

  simulation_time = simulation_time + dt;
  
  // Compute line length
  line_length = line_length + reel_speed*dt;
  if (line_length<2)
  {
    reel_speed = 0;
    line_length = 2;
  }
  base_position = base_position.add(base_velocity.multiplyScalar(dt));
  computeForces();
  
  torque_at_base.crossVectors(kite_position.sub(base_position), Fsum);
  //console.log("torque", torque_at_base);
  inv_kite_mass = 1./kite_mass;
  
  if (true==is1dof){
    //console.log(pqr.x);
    kite_angular_acceleration = 1/(kite_mass*line_length^2)*torque_at_base.x;
    //kite_angular_acceleration = Math.max(-60000, Math.min(kite_angular_acceleration,60000));
    pqr.x = pqr.x+kite_angular_acceleration*dt;
    // Saturate to avoid divergences
    //pqr.x = Math.max(-60, Math.min(pqr.x,60));
    elevation = elevation+pqr.x*dt;
    kite_position.set(0, Math.cos(elevation), Math.sin(elevation)).multiplyScalar(line_length);
    kite_velocity.set(0, -Math.sin(elevation), Math.cos(elevation)).multiplyScalar(line_length*pqr.x);
    line_tension = Faero.y*Math.cos(elevation) +  (Faero.z-kite_mass*g)*Math.sin(elevation) + kite_mass*pqr.x*pqr.x*line_length;
  }
  else
  {
    kite_velocity =kite_velocity.add(Fsum.multiplyScalar(inv_kite_mass*dt));
    // Saturate to avoid divergences
    kite_velocity.set(Math.max(-100, Math.min(kite_velocity.x,100)), Math.max(-100, Math.min(kite_velocity.y,100)), Math.max(-100, Math.min(kite_velocity.z,100)));
  
    kite_position = kite_position.add(kite_velocity.multiplyScalar(dt));
    elevation = Math.atan2(kite_position.z, kite_position.y);
  }
  
  
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
    if (reel_speed==0)
    {console.log(myRange.value)
    line_length = 1*myRange.value;
    }
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
    wind_speed = myOutput.value;
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
    myOutput.value = Math.round(pqr.x*line_length*10)/10;
  }
