width = 15;
beam_size = 10;
beam_thickness = 5;
thickness = 2;
bridge_length = 32;
potentiometer_width = 17;
total_length = bridge_length+2*width+2*thickness;

module bottom_beam(){
difference(){
union(){
		difference(){
		union(){
			cube(size = [total_length, beam_size, beam_thickness], center=true);
			
			translate([total_length/2 - thickness/2,0,0]) cube(size = [thickness, beam_size+2*thickness, beam_thickness], center=true);
			translate([-total_length/2 + thickness/2,0,0]) cube(size = [thickness, beam_size+2*thickness, beam_thickness], center=true);
			
			translate([total_length/2 - thickness/2-thickness-width,0,0]) cube(size = [thickness, beam_size+2*thickness, beam_thickness], center=true);
			translate([-total_length/2 + thickness/2+thickness+width,0,0]) cube(size = [thickness, beam_size+2*thickness, beam_thickness], center=true);
		}

	//Remove space to save weight?
		translate([bridge_length/2+width/2,0,0]) cube(size = [width, beam_size-2*thickness, 2*beam_thickness], center=true);
		translate([-bridge_length/2-width/2,0,0]) cube(size = [width, beam_size-2*thickness, 2*beam_thickness], center=true);
	}
		minkowski(){
			cube(size = [potentiometer_width, potentiometer_width, beam_thickness/2], center=true);
			cylinder(r= 2, h=beam_thickness/2,center=true);
}
	}
	cube(size = [potentiometer_width, potentiometer_width, beam_thickness*2], center=true);
	translate([0 , potentiometer_width/2],0) cube(size = [potentiometer_width, potentiometer_width, beam_thickness*2], center=true);
}
	
}
module top_beam(){
difference(){
	union(){
			difference(){
		union(){
			cube(size = [bridge_length+2*thickness, beam_size, beam_thickness], center=true);
			

			
			translate([total_length/2 - thickness/2-thickness-width,0,0]) cube(size = [thickness, beam_size+2*thickness, beam_thickness], center=true);
			translate([-total_length/2 + thickness/2+thickness+width,0,0]) cube(size = [thickness, beam_size+2*thickness, beam_thickness], center=true);
		}

	//Remove space to save weight?
		translate([bridge_length/2+width/2,0,0]) cube(size = [width, beam_size-2*thickness, 2*beam_thickness], center=true);
		translate([-bridge_length/2-width/2,0,0]) cube(size = [width, beam_size-2*thickness, 2*beam_thickness], center=true);
	}
		cylinder(r= 5+thickness, h= beam_thickness, center=true);
	}
	cylinder(r= 5, h= 2* beam_thickness, center=true);
}
}

translate([0, 0, beam_thickness+1]) top_beam();
bottom_beam();

translate([0, 0, 2* beam_thickness+2]) difference(){
 	cylinder(r=3+thickness,h=beam_thickness, center=true);
	cylinder(r=3, h=2*beam_thickness, center=true);
}


