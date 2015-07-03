use <compass.scad>
use <arduino_footprint.scad>
use <battery_footprint.scad>

thickness = 5;

difference()
{
	union()
	{
		arm();
		translate([20,0,0]) cube(size=[50,150,thickness], center=true);
	}
	translate([20,-45,0]) nano_connector_footprint(2*thickness);
	translate([20,5,0]) 9V_battery();
	translate([0,5,0]) cube(size=[3,12,2*thickness], center=true);
	translate([40,5,0]) cube(size=[3,12,2*thickness], center=true);
	translate([0,-45,0]) cube(size=[3,12,2*thickness], center=true);
	translate([40,-45,0]) cube(size=[3,12,2*thickness], center=true);
}

