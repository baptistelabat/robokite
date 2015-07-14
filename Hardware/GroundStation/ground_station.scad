//rotate([90,0,0]) union(){
//	rotate_extrude(convexity = 10)
//	translate([6, 0, 0])
//	circle(r = 1);
//}
rotate([90,0,0]) union(){
	linear_extrude(height = 2, center = false, convexity = 10, twist = 360, $fn = 100)
	translate([6, 0, 0])
	scale([1,1,1]) circle(r=1);
}
rotate([90,0,0])
minkowski(){
rotate([0,0,90]) translate([6,0,0]) sphere(r=1);
rotate([0,0,70]) translate([6,0,0]) sphere(r=1);
}

winch();
translate([480, 0,0]) rotate([0, 0, 180]) winch();
translate([480/2,0,-40/2-20/2]) cube(size=[480+100, 300,20], center=true);
module pulley(){
	difference(){
	cylinder(r=6, h =10, center= true);
	cylinder(r=5, h =11, center= true);
	
	}
	difference(){
	cylinder(r=15, h =10, center= true);
	cylinder(r=7.5, h =11, center= true);
	
	}
}

module winch(){
	rotate([90, 0,0]) union(){
		translate([0,0, 50])
		union(){
			pulley();
			pulley_support();
		}
		translate([0,0, -50])
		union(){
			pulley();
			pulley_support();
		}
		translate([0,0,0])  cylinder(r=5, h=150, center=true);
	
	}
}

module pulley_support(){
difference(){
translate([25, 0,0]) cube(size=[100, 40,10], center=true);
cylinder(r=16, h =11, center= true);
translate([-15,0,0]) cube(size=[25,12,11], center=true);
}
}