thickness = 2;
width=85;
length=133-30;
nano_length_max=43.6;
nano_length_min = 37.1;
nano_width_max = 22.7;
nano_internal_width = 9;
nano_thickness=7;
9V_width=26.2;
9V_length=45;
transmitter_width = 30;
transmitter_length= 45;
IMU_screw_spacing_width = 12.1;
IMU_screw_spacing_length = 16.3;
corner_radius=5;

module support()
{
	difference()
	{
		minkowski(){
		translate([0,-15,0]) cube(size=[width-2*corner_radius,length-2*corner_radius,thickness/3.], center=true);
		cylinder(h=thickness/3., r=corner_radius);
}
		translate([10,12,0]) difference(){
			cube(size=[nano_width_max,nano_length_min,2*thickness], center=true);
		 	cube(size=[nano_internal_width,nano_length_min,2*thickness], center=true);
	}
		translate([-23,8,0]) cube(size=[9V_width,9V_length,2*thickness], center=true);
		//translate([10,-45,0]) cube(size=[transmitter_length,transmitter_width,2*thickness], center=true);
	translate([0,-60,0]) cube(size=[12,3,2*thickness], center=true);
	translate([0,-60 +transmitter_width ,0]) cube(size=[12,3,2*thickness], center=true);
	
	
	// Holes to screw drotek IMU board	
		translate([-30, -50,0]) cylinder(r=0.3, h=2*thickness, center=true);
		translate([-30+IMU_screw_spacing_length, -50+IMU_screw_spacing_width,0]) cylinder(r=0.3, h=2*thickness, center=true);
		translate([-30+IMU_screw_spacing_length, -50,0]) cylinder(r=0.3, h=2*thickness, center=true);
	}
}
projection() support();