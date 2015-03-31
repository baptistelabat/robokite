tube_diameter = 28;
margin = 1 ;
thickness	    =1;
width = 15;
beam_size = 10;
beam_thickness = 5;
clip_width=10;

difference(){
	union(){
		// Cylinder
		rotate([90,0,0]) cylinder(r=tube_diameter/2+thickness+margin, h = width, center=true);

		// Top flat part
		translate([0, 0, beam_thickness]) cube(size=[beam_size+2*thickness, width, tube_diameter+2*thickness], 	center = true);
		
		// Part to block strap
		translate([0, width/2-(width-clip_width)/4, 0]) hull(){
		rotate([90,0,0]) cylinder(r=tube_diameter/2+thickness+margin+1, h = (width-clip_width)/2, center=true);
		translate([0, 0, beam_thickness+1]) cube(size=[beam_size+2*thickness, (width-clip_width)/2, tube_diameter+2*thickness], center = true);
		}

		// Second part to block strap
		translate([0, -width/2+(width-clip_width)/4, 0]) hull(){
		rotate([90,0,0]) cylinder(r=tube_diameter/2+thickness+margin+1, h = (width-clip_width)/2, center=true);
		translate([0, 0, beam_thickness+1]) cube(size=[beam_size+2*thickness, (width-clip_width)/2, tube_diameter+2*thickness], center = true);
		}
	}

	// Place for beam
	translate([0, 0, tube_diameter/2+thickness*2+beam_thickness/2+1]) cube(size=[beam_size, 2*width, beam_thickness+2], center = true);

	// Cylinder corresponding to space for bar
	rotate([90,0,0]) cylinder(r=tube_diameter/2+margin, h = 2*width, center=true);

	// Cube to hide bottom
	translate([0, 0, -tube_diameter*5/8]) cube(size=[2*tube_diameter, 2*width, tube_diameter*2], center=true);

}