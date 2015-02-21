module support(){
	distance_between_pulleys = 100;
	margin_around_holes = 7;
	hole_diameter = 5;
	pulley_diameter = 1;
	thickness = 5;
	n_tuning_holes = 3;
	distance_between_tuning_holes = 20;
	
	// Support with different holes to tune the position of //the fixation of the gauge sensor
	difference(){
		translate([0, n_tuning_holes*distance_between_tuning_holes/2, 0]) cube(size =[2*margin_around_holes + hole_diameter, 2*margin_around_holes + hole_diameter + n_tuning_holes*distance_between_tuning_holes, thickness], center = true);
		
		// Tuning holes
		for (i = [0:n_tuning_holes]){
			translate([0, i*distance_between_tuning_holes, 0]) cylinder(r=2.5,h=thickness, $fs=1, center = true);
		}
	}
	
	// Support for the two pulleys
	difference(){
		cube(size=[distance_between_pulleys+2*margin_around_holes+hole_diameter, 2*margin_around_holes + hole_diameter, thickness], center = true);
	
		// Hole for first pulley axes
		translate([distance_between_pulleys/2, 0, 0]) cylinder(r=2.5,h=thickness, $fs=1, center = true);
		// Hole for second pulley axes
		translate([-distance_between_pulleys/2, 0, 0]) cylinder(r=2.5,h=thickness, $fs=1, center = true);
	}
}

projection() support();

