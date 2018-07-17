$fn=200;
insert(31,35,21.5,3,28);
//translate([0,0,4])
//nut(42);


module nut(diameter){
    difference(){
        union(){
            translate([0,0,2])
            cylinder(d=diameter,h=14-4,$fn=6);
        }union(){
            thread(31,14,2,3.8,1.3); // inner
        }
    }
}

module insert(diameter,lowerDiameter,height,lowerHeight,outerBottleDiameter) {
    difference(){
        union(){
            cylinder(d=diameter,h=1.6);
            cylinder(d=diameter-(1.3*2),h=height);
            translate([0,0,-lowerHeight])
            cylinder(d=lowerDiameter,h=lowerHeight);
            translate([0,0,1.6])
            thread(diameter-(1.3*2),12,2,3.8,1.3); // inner
        }union(){
            translate([0,0,-2])
            thread(25,14,2,3.8,1.3); // Club Mate
            translate([0,0,-lowerHeight-0.01])
            cylinder(d=outerBottleDiameter,h=lowerHeight+0.01);
            translate([0,0,10])
            cylinder(d=outerBottleDiameter-4,h=15);
        }
    }
}

// inner diameter (without blades), height of the thread, thickness of the blades, pitch (mm/height per rev), depht of the blades
module thread(diameter,height,thickness,pitch,depth){
    res = $fn;
    angle = 40;
    difference(){
        union(){
         cylinder(d=diameter,h=height);   
         for(i = [0:1:(res*((height-thickness)/pitch))]){
             hull(){
                 block(i,diameter,height,thickness,pitch,depth,res);
                 block(i-1,diameter,height,thickness,pitch,depth,res);
             }
         }
        }union(){

        }
    }
}

module block(cnt,diameter,height,thickness,pitch,depth,res,angle = 0){
    rotate([0,0,cnt*360/res])
    translate([diameter/2,0,(pitch/res)*cnt])
    rotate([0,0,12])
    cube([depth,360/res,thickness]);
}