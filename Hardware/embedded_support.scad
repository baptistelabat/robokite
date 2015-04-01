RF_module_length  = 160;
9V_battery_length = 50;
length_for_arduino_nano = 30;
total_length = RF_module_length +9V_battery_length+length_for_arduino_nano; 
width = 30; // Based on arduino nano length between "legs"
thickness = 5;

projection() embedded_support();

module embedded_support(){
	difference(){
		union(){
			// Main support
			cube(size = [total_length +20, width, thickness], center = true);
			
			// Right plate to attach strap
			translate([total_length/2+20/2-30/2,0,0]) cube(size = [30, width+20, thickness], center = true);
	
			// Left plate to attach strap
			translate([-total_length/2-20/2+30/2,0,0]) cube(size = [30, width+20, thickness], center = true);
		}
				// Right holes to attach strap
			translate([total_length/2+20/2-30/2,17.5,0]) cube(size = [20, 5, 2*thickness], center = true);
			translate([total_length/2+20/2-30/2,-17.5,0]) cube(size = [20, 5, 2*thickness], center = true);
	
			// Left holes to attach strap
			translate([-total_length/2-20/2+30/2,17.5,0]) cube(size = [20, 5, 2*thickness], center = true);
			translate([-total_length/2-20/2+30/2,-17.5,0]) cube(size = [20, 5, 2*thickness], center = true);
	}
}