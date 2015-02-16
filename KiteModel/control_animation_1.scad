
distance_between_fixed_pulleys = 30;
distance_between_lines = 30;
control_length_left = 20 + 20*sin($t*360);
control_length_right = 20 + 20*sin($t*360*2);
L=300;


// Left line
hull()
{
translate([ -distance_between_lines,0,0]) sphere(r=1);
translate([ -control_length_left -distance_between_lines, distance_between_fixed_pulleys/2., 0]) sphere(r=1);
};
hull()
{
translate([ -control_length_left -distance_between_lines, distance_between_fixed_pulleys/2., 0]) sphere(r=1);
translate([ -distance_between_lines, distance_between_fixed_pulleys, 0]) sphere(r=1);
};
hull()
{
translate([ -distance_between_lines, distance_between_fixed_pulleys, 0]) sphere(r=1);
translate([ -distance_between_lines, distance_between_fixed_pulleys + 300 -2*sqrt(pow(distance_between_fixed_pulleys/2., 2)+ pow(control_length_left, 2)) , 0]) sphere(r=1);
};

// Central line
hull()
{
translate([ -0*distance_between_lines,0,0]) sphere(r=1);
translate([ -0*distance_between_lines, distance_between_fixed_pulleys, 0]) sphere(r=1);
};
hull()
{
translate([ -0*distance_between_lines, distance_between_fixed_pulleys, 0]) sphere(r=1);
translate([ -0*distance_between_lines, distance_between_fixed_pulleys + L -2*sqrt(pow(distance_between_fixed_pulleys/2., 2)) , 0]) sphere(r=1);
};

// Right line
hull()
{
translate([ distance_between_lines,0,0]) sphere(r=1);
translate([ control_length_right +distance_between_lines, distance_between_fixed_pulleys/2., 0]) sphere(r=1);
};
hull()
{
translate([ control_length_right +distance_between_lines, distance_between_fixed_pulleys/2., 0]) sphere(r=1);
translate([ distance_between_lines, distance_between_fixed_pulleys, 0]) sphere(r=1);
};
hull()
{
translate([ distance_between_lines, distance_between_fixed_pulleys, 0]) sphere(r=1);
translate([ distance_between_lines, distance_between_fixed_pulleys + 300 -2*sqrt(pow(distance_between_fixed_pulleys/2., 2)+ pow(control_length_right, 2)) , 0]) sphere(r=1);
};