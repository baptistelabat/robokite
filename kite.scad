
lineLength=35;
bearing =0 + sin($t*360)*45;
elevation = 30+sin(-$t*360*2)*15;
yaw = bearing-sin($t*360)*30;
drift=5;
roll = sin(360*$t+90+drift)*120;
angleOfAttack=15;
pitch=elevation + angleOfAttack;
useNED=-1;
rotate(a=bearing, v=[0,0,1*useNED]) rotate(a=elevation, v=[0,1*useNED,0]){
	translate(v=[lineLength/2,0,0]) cube(size=[lineLength,0.1,0.1], center=true);
	translate(v=[lineLength,0,0]) 
 	  rotate(a=-1*elevation, v=[0,1*useNED,0]) rotate(a=-1*bearing, v=[0,0,1*useNED])
	  rotate(a=yaw ,v=[0,0,1*useNED]) rotate(a=pitch, v=[0,1*useNED,0]) rotate(a=roll, v=[1,0,0]) 
       color("red") 
//cube(size=[0.1,,3,1],center=true); }
polygon(points=[[2,0],[2,1], [1.5,2],[0,3],[1.3,2],[1.8,1], [1.8,0],[1.8,-1],[1.3,-2],[0,-3],[1.5,-2],[2,-1]], paths=[[0,1,2,3,4,5,6,7,8,9,10,11]]);}
//import("skpfile.stl");
