$fn=20;

X=14+2*1+0.5;
Y=25+2*1;
Z=6.1*2+1;
E=1;
F=9+2*1;
RT=1;
XT=9/2;
ZT=2.54/2;
DZ=E;

difference() {
	translate([0,0, -E]) cube([X,Y,Z],center=true);
	translate([0,0,0]) cube([X-2*E,Y-2*E,Z],center=true);
	translate([0,0,-Z/2]) cube([5,20,Z],center=true);
	for (i=[-1,1], j=[-1,1], k=[-1,1]) {
		translate([i*XT,Y/2,j*ZT+k*6.1/2]) rotate([90,0,0]) cylinder(r=RT,h=Y+2);
	}	
}