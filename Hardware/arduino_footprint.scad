module nano_connector_footprint(thickness)
{
	nano_length_max=43.6;
	nano_connectors_length = 37.1;
	nano_width_with_connectors = 22.5;
	nano_internal_width = 10;

	difference(){
			cube(size=[nano_width_with_connectors,nano_connectors_length,thickness], center=true);
		 	cube(size=[nano_internal_width,nano_connectors_length,2*thickness], center=true);
	}
}

nano_connector_footprint(5);

