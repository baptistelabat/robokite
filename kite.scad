yaw = 90;
roll = 0;
pitch=0;
lineLength=20;
bearing =90;
elevation = 45;
useNED=-1;
 rotate(a=bearing, v=[0,0,1*useNED]) rotate(a=elevation, v=[0,1*useNED,0])  translate(v=[lineLength,0,0])  rotate(a=-1*elevation, v=[0,1*useNED,0]) rotate(a=-1*bearing, v=[0,0,1*useNED])   rotate(a=yaw ,v=[0,0,1*useNED]) rotate(a=pitch, v=[0,1*useNED,0]) rotate(a=roll, v=[1,0,0]) cube(size=[1,3,0.1]); 