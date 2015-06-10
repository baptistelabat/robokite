RF_module_length  = 30;
9V_battery_length = 50;
length_for_arduino_nano = 30;
total_length = RF_module_length +9V_battery_length+length_for_arduino_nano; 
width = 15; // Based on arduino nano length between "legs"
thickness = 5;

projection() embedded_support();

module embedded_support(){
	difference(){
		union(){
			translate([total_length/2+10,0,0]) cylinder(r=15, h=thickness, center=true);
			// Main support
			cube(size = [total_length +20, width, thickness], center = true);
			translate([30,0,0]) cube(size = [length_for_arduino_nano, length_for_arduino_nano, thickness], center = true);
			
			// Right plate to attach strap
			translate([total_length/2+20/2-30/2,0,0]) cube(size = [30, width+20, thickness], center = true);
	
			// Left plate to attach strap
			translate([-total_length/2-20/2+30/2,0,0]) cube(size = [30, width+20, thickness], center = true);
		}
				// Right holes to attach strap
			translate([total_length/2+20/2-30/2,7.5,0]) cube(size = [20, 5, 2*thickness], center = true);
			translate([total_length/2+20/2-30/2,-7.5,0]) cube(size = [20, 5, 2*thickness], center = true);
	
			// Left holes to attach strap
			translate([-total_length/2-20/2+30/2,7.5,0]) cube(size = [20, 5, 2*thickness], center = true);
			translate([-total_length/2-20/2+30/2,-7.5,0]) cube(size = [20, 5, 2*thickness], center = true);
			translate([total_length/2+10,0,0]) cylinder(r=2.5, h=thickness, center=true);

	}
}