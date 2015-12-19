pot_radius=5.90/2;
meplat = 1.5;
width=12.5;
length = 13.2;
screw_radius = 8.6/2;
pot_length = 12.8;
screw_length = 6.9;
thickness = 3;
axis_radius = 5.5;

wheel_radius = width/2+thickness+0.5;

module pot(mep){
translate([0,0,-1.5-1]) cube(size=[width, length, 5], center=true);
translate([0,0, -1])cylinder(r= screw_radius, h=screw_length+1);
difference(){
		translate([0,0,screw_length-1]) cylinder(r=pot_radius, h=pot_length+2);
		translate([2*pot_radius-mep,0,50+screw_length]) cube([2*pot_radius	, 100,100], center=true);
}
}
//pot(meplat);
difference(){
union(){
translate([0,0,-1.5+1]) cube(size=[width+thickness, length+thickness, 5], center=true);
cylinder(r= screw_radius+thickness, h=screw_length);
difference(){
		translate([0,0,screw_length-thickness]) cylinder(r=wheel_radius+thickness, h=pot_length+thickness);
		translate([50,0,screw_length+(pot_length-thickness)/2]) cube([100, 100,pot_length-thickness], center=true);
	
		translate([0,0,screw_length]) cylinder(r=wheel_radius+0.5, h=pot_length-thickness);
}
}
pot(0);
	translate([wheel_radius+axis_radius-0.1,0,0]) cylinder(r=axis_radius, h=100, center=true);
}



translate([30,0,0])
difference(){
	translate([0,0, screw_length+1]) cylinder(r=wheel_radius, h=pot_length-5);
pot(meplat);
	}

