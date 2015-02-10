width = 97; // External width (except clipping mechanism)
height = 142; // External height (except clipping mechanism)
corner_radius = 14; // Round corner at the bottom corners
top_width = 114; // Clipping mechanism width
top_height = 20; // Clipping mechanism height
thickness = 2*0.7; // Thickness of clipping mechanism
clip_height = 12;

//Main pocket
difference()
{
	union(){
	cube(size = [width,height,thickness], center = true);
	translate([-width/2, 0,0]) cylinder(r=20, h=thickness, center = true);
	translate([width/2, 0,0]) cylinder(r=20, h=thickness, center = true);
	}
	cube(size = [width-20,height-24,thickness], center = true);
}

// Existing top clip
//translate([0, height/2 + top_height/2, 0]) cube(size = [top_width,top_height,thickness], center = true);


clip_width = width +20;
// Top clip support
translate([0, height/2 - clip_height/2, 0]) cube(size = [clip_width, clip_height, thickness], center = true);

// Top clip 
translate([-clip_height, 0 , 0]) cube(size = [clip_height, clip_width, thickness], center = true);

// Bottom clip support
translate([0, -height/2+clip_height/2, 0]) cube(size = [clip_width, clip_height, thickness], center = true);

// Bottom clip
translate([clip_height, 0, 0]) cube(size = [clip_height, clip_width, thickness], center = true);




