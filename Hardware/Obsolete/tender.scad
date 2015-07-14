thickness=5;
projection() union(){
	translate([20 ,0,0]) cube(size=[35, 50, thickness], center=true);
	translate([0,0,0]) tender();
}
module tender()
{
difference(){
	cube(size=[20, 45, thickness], center=true);
	translate([2, 0, 0]) cylinder(r=2.5, h=2*thickness, center = true);
		translate([0-4, 1, 0]) rotate([0,0, -20]) cube(size=[15, 4, 2*thickness], center=true);
		translate([0+2, 13, 0]) cylinder(r=2.5, h=2*thickness, center = true, $fs=1);
		translate([0-4, 13+1, 0]) rotate([0,0, -20]) cube(size=[15, 4, 2*thickness], center=true);
		translate([0+2, -13, 0]) cylinder(r=2.5, h=2*thickness, center = true, $fs=1);
		translate([0-4, -13-1, 0]) rotate([0,0, 20]) cube(size=[15, 4, 2*thickness], center=true);
		translate([0-5, -6.5, 0]) cylinder(r=2.5, h=2*thickness, center = true, $fs=1);
	
	}
}