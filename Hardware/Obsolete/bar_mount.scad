tube_diameter = 28;
space_margin = 1;
thickness = 2;
screw_diameter = 5;
screw_head_thickness = 2;

a = tube_diameter + 2* space_margin +2*thickness;
difference(){
	union(){
		//Main part
		difference(){
		cube(size=[a,a,a+4], center = true);
		translate([a/4, 0,0]) cube(size=[4,2*a, 2*a], center = true);
		translate([-a/4, 0,0]) cube(size=[4,2*a, 2*a], center = true);
		}
		//Screw tube
		translate([0, a/2+screw_diameter/2,0]) difference(){
			cylinder(r = screw_diameter/2+thickness, h = a, center=				true);
			cylinder(r = screw_diameter/2, h = a, center=true);
		}
		//Second screw tube
		translate([0, -a/2-screw_diameter/2,0]) difference(){
			cylinder(r = screw_diameter/2+thickness, h = a, center=				true);
			cylinder(r = screw_diameter/2, h = a, center=true);

		}
	rotate([0,90,0])  cylinder(r=tube_diameter/2. + space_margin+thickness, h= a, center = 	true);
	}
	rotate([0,90,0])  cylinder(r=tube_diameter/2. + space_margin, h= 2*a, center = true);
   cube(size =[2*a, 2*a,tube_diameter/5.], center = true);
	translate([0,-a/2-screw_diameter/2,a/2]) cylinder(r = screw_diameter/2+thickness+0.1, h = 2*screw_head_thickness, center=				true);
	translate([0,-a/2-screw_diameter/2,-a/2]) cylinder(r = screw_diameter/2+thickness+0.1, h = 2*screw_head_thickness, center=				true);
	translate([0,+a/2+screw_diameter/2,a/2]) cylinder(r = screw_diameter/2+thickness+0.1, h = 2*screw_head_thickness, center=				true);
	translate([0,+a/2+screw_diameter/2,-a/2]) cylinder(r = screw_diameter/2+thickness+0.1, h = 2*screw_head_thickness, center=				true);
	
	// To erase one half
	translate([0, 0, a]) cube([2*a,2*a,2*a], center = true);
}


