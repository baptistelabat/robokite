distance_between_pulleys = 100;
margin_around_holes = 5;
hole_diameter = 5;
pulley_diameter = 1;
thickness = 5;
n_tuning_holes = 3;

cube(size =[2*margin_around_holes + hole_diameter, 2*margin_around_holes + hole_diameter + 100], center = true);
//cube(size=[distance_between_pulleys+2*margin_around_holes+hole_diameter, 2*margin_around_holes + hole_diameter, thickness], center = true);

