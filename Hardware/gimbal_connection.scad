use <MCAD/nuts_and_bolts.scad>

guiding_length = 10;
x = 2;
space = 15;


difference(){
union(){
	hull(){
	translate([space +guiding_length/2,0,0]) cube(size=[15,10,guiding_length], center = true);
	translate([x, 0, x]) sphere(3);
	}

	hull(){
		translate([0,0,space+guiding_length/2]) rotate([0,90,0])cube(size=[15,10,guiding_length], center = true);
		translate([x, 0, x]) sphere(3);
	}
}

	translate([space +guiding_length/2,0,0]) cylinder(r=3.25, h=10*guiding_length, center = true);
	translate([0,0,space+guiding_length/2]) rotate([0,90,0])cylinder(r=3.25, h=10*guiding_length, center = true);

//space for nut
	rotate([0,-90,0])
union(){translate([space +guiding_length/2+4.5,0, 0]) rotate([0,90,0]) translate([0,0,-1.25])  nutHole(3);
 translate([space +guiding_length/2+2.5,0, 0]) rotate([0,90,0]) translate([0,0,-1.25]) nutHole(3);

	translate([space +guiding_length/2+5,0, 0]) rotate([0,90,0]) cylinder(r=1.5, h=15, center=true);}

	translate([space +guiding_length/2+4.5,0, 0]) rotate([0,90,0]) translate([0,0,-1.25])  nutHole(3);
	translate([space +guiding_length/2+2.5,0, 0]) rotate([0,90,0]) translate([0,0,-1.25])  nutHole(3);
	translate([space +guiding_length/2+5,0, 0]) rotate([0,90,0]) cylinder(r=1.5, h=15, center=true);

//Cube to visualize
	//cube(space +guiding_length/2+7);

}

//cube(size=[sqrt(guiding_length
