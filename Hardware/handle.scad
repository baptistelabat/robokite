module handle(){
	thickness = 5;
	difference(){
		minkowski(){
		 linear_extrude(height=thickness/3)
		     polygon(points=[[0,0],[150,30],[300,0],[280,15],[280,85],[300,100],[270,120], [30,120],[0,100], [20,85],[20,15]],paths=[ [0,1,2,3,4,5,6,7,8,9,10] ]);
		cylinder(r=10, h=thickness/3);
		}
		hull(){
			translate([70,90,0]) cylinder(r=15, h= 10*thickness, center= true);
			translate([230,90,0]) cylinder(r=15, h= 10*thickness, center= true);
		}
		translate([150, 70,0]) cube(size=[2, 20, 10*thickness], center = true);
	}
}

projection() handle();