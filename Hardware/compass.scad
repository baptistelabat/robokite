//compasss
module arm(){
	thickness = 5;
	hole_diameter=5;
	plate_diameter = 30;
	min_thickness = 5;
	distance_between_axes = 115;
	difference(){
		union(){
			translate([0, distance_between_axes/2,0]) cylinder(r=plate_diameter/2., h=thickness, center= true, $fs=1);
			translate([0, -distance_between_axes/2,0]) cylinder(r=plate_diameter/2., h=thickness, center= true, $fs=1);
			cube(size=[hole_diameter + 2*min_thickness, distance_between_axes + 2*min_thickness + hole_diameter,thickness], center = true);
		}
		translate([0, distance_between_axes/2,0]) cylinder(r=hole_diameter/2., h=thickness, center= true, $fs=1);
		translate([0, -distance_between_axes/2,0]) cylinder(r=hole_diameter/2., h=thickness, center= true, $fs=1);
	}
}
for (z = [-1, 1]) 
{}
arm();
//projection() union(){for (i=[0:5]){translate([35*i, 0,0]) arm();}};




