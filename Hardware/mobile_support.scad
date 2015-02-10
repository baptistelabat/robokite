width = 97; // External width (except clipping mechanism)
height = 146; // External height (except clipping mechanism)
corner_radius = 14; // Round corner at the bottom corners
top_width = 114; // Clipping mechanism width
top_height = 20; // Clipping mechanism height
thickness = 5; // Thickness
clip_height = 14;

//Main pocket
difference()
{
	union(){
		cube(size = [width,height,thickness], center = true);
		difference(){
			translate([-width/2, 0,0]) cylinder(r=20, h=thickness, center = true);
			translate([-width/2-5, 0,0]) cylinder(r=2.5, h=thickness, center = true);
			translate([-width/2-1.5, -10,0]) cube(size =[3,26,thickness], center = true);
		}
		difference(){
			translate([width/2, 0,0]) cylinder(r=20, h=thickness, center = true);
			translate([width/2+5, 0,0]) cylinder(r=2.5, h=thickness, center = true);
			translate([width/2+1.5, -10,0]) cube(size =[3,26,thickness], center = true);
}
	}
	cube(size = [width-20,height-24,thickness], center = true);
}

// Existing top clip
//translate([0, height/2 + top_height/2, 0]) cube(size = [top_width,top_height,thickness], center = true);


clip_width = width +20;
// Top clip support
difference(){
	translate([0, height/2 - clip_height/2, 0]) cube(size = [clip_width, clip_height, thickness], center = true);
	translate([-clip_width/2+7, height/2- clip_height/2, 0]) cylinder(r=2.5, h=thickness, center = true);
	translate([+clip_width/2-7, height/2- clip_height/2, 0]) cylinder(r=2.5, h=thickness, center = true);
}

// Top clip
difference()
{
	translate([-clip_height, 0 , 0]) cube(size = [clip_height, clip_width, thickness], center = true);
	translate([-clip_height, clip_width/2-7, 0]) cylinder(r=2.5, h=thickness, center = true);
	translate([-clip_height, -clip_width/2+7, 0]) cylinder(r=2.5, h=thickness, center = true);
}

// Bottom clip support
difference(){
	translate([0, -height/2+clip_height/2, 0]) cube(size = [clip_width, clip_height, thickness], center = true);
	translate([-clip_width/2+7, -height/2+ clip_height/2, 0]) cylinder(r=2.5, h=thickness, center = true);
	translate([+clip_width/2-7, -height/2+ clip_height/2, 0]) cylinder(r=2.5, h=thickness, center = true);
}
// Bottom clip
difference(){
 translate([clip_height, 0, 0]) cube(size = [clip_height, clip_width, thickness], center = true);
	translate([clip_height, clip_width/2-7, 0]) cylinder(r=2.5, h=thickness, center = true);
	translate([clip_height, -clip_width/2+7, 0]) cylinder(r=2.5, h=thickness, center = true);
}




