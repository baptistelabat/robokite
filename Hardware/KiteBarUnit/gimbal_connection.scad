guiding_length = 10;
x = 2;
space = 16;
potentiometer_diameter = 6.6;
alpha = 2/3;

total_length = guiding_length*3/2+space;

// Bottom part
difference(){
	union(){
		cube(size=[total_length,guiding_length,guiding_length], center = true);
	}

	translate([total_length/2-guiding_length/2,0,0]) cylinder(r=potentiometer_diameter/2, h=10*guiding_length, center = true);
	translate([-total_length/2+guiding_length/2+guiding_length*(1-alpha)/2,0,0]) cube(size=[alpha*guiding_length, alpha*guiding_length, 2*guiding_length],center = true) ;
}


// Top part to be inserted at 90deg
translate([0,1.5*guiding_length,0])
union(){
difference(){
	union(){
		cube(size=[total_length,guiding_length,guiding_length], center = true);
	}

	translate([total_length/2-guiding_length/2,0,0]) cylinder(r=potentiometer_diameter/2, h=10*guiding_length, center = true);

	difference(){
	translate([-total_length/2+guiding_length/2-1,0,0]) cube(size=[guiding_length+2, guiding_length+2, 2*guiding_length],center = true) ;
	translate([-total_length/2+guiding_length/2,0,-guiding_length*(1-alpha)/2]) cube(size=[guiding_length, alpha*guiding_length, alpha*guiding_length],center = true) ;
}
}
}



