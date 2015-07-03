use <compass.scad>
use <arduino_footprint.scad>
use <battery_footprint.scad>


module support_arm()
{
	thickness = 5;
	
	difference()
	{
		union()
		{
			arm();
			minkowski()
			{
				cylinder(r=5, h=thickness/3.0);
				translate([20,0,0]) cube(size=[50,150,thickness/3.0], center=true);
			}		
		}
		translate([20,-45,0]) nano_connector_footprint(2*thickness);
		translate([20,5,0]) 9V_battery();
		translate([0,5,0]) cube(size=[3,12,2*thickness], center=true);
		translate([40,5,0]) cube(size=[3,12,2*thickness], center=true);
		translate([0,-45,0]) cube(size=[3,12,2*thickness], center=true);
		translate([40,-45,0]) cube(size=[3,12,2*thickness], center=true);
	}
}
projection() support_arm();

