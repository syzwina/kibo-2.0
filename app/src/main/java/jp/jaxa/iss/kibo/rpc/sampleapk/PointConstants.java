package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.Arrays;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public final class PointConstants {

    //recalculating optimal coords using POINT4 and TARGET4 as reference (yz axis)
    private static final double Y_COORDS_OFFSET = -0.044528; // -6.7185--6.673972 //to use= target + offset, // facing the target, it move to the right, ie -ve of y-axis.
    private static final double Z_COORDS_OFFSET = 0.08509; // 5.1804-5.09531 //to use= target + offset, // to move down, +ve of z-axis

    //Old Point refers to original point given in rulebook
    static final Point START_COORDS = new Point(9.815, -9.806, 4.293);
    static final Point GOAL_COORDS = new Point(11.143, -6.7607, 4.9654);

    static final Point OLD_POINT1_COORDS = new Point(11.2746, -9.92284, 5.2988);
    static final Point OLD_POINT2_COORDS = new Point(10.612, -9.0709, 4.48);
    static final Point OLD_POINT3_COORDS = new Point(10.71, -7.7, 4.48);
    static final Point POINT4_COORDS = new Point(10.51, -6.7185, 5.1804);
    static final Point OLD_POINT5_COORDS = new Point(11.114, -7.9756, 5.3393);
    static final Point OLD_POINT6_COORDS = new Point(11.355, -8.9929, 4.7818);
    static final Point POINT7_COORDS = new Point(11.369, -8.5518, 4.48);

    static final Point TARGET1_COORDS = new Point(11.2625, -10.58, 5.3625);
    static final Point TARGET2_COORDS = new Point(10.513384, -9.085172, 3.76203);
    static final Point TARGET3_COORDS = new Point(10.6031, -7.71007, 3.76093);
    static final Point TARGET4_COORDS = new Point(9.866984, -6.673972, 5.09531);
    static final Point TARGET5_COORDS = new Point(11.102, -8.0304, 5.9076);
    static final Point TARGET6_COORDS = new Point(12.023, -8.989, 4.8305);
    static final Point QR_CODE_COORDS = new Point(11.381944, -8.566172, 3.76203);

    // new POINT_COORDS optimized from POINT4 and target4 reference
    static final Point POINT1_COORDS = new Point(TARGET1_COORDS.getX() - Y_COORDS_OFFSET, OLD_POINT1_COORDS.getY(), TARGET1_COORDS.getZ() + Z_COORDS_OFFSET);
    static final Point POINT2_COORDS = new Point(TARGET2_COORDS.getX() - Y_COORDS_OFFSET, TARGET2_COORDS.getY() - Z_COORDS_OFFSET, OLD_POINT2_COORDS.getZ());
    static final Point POINT3_COORDS = new Point(TARGET3_COORDS.getX() + Z_COORDS_OFFSET, TARGET3_COORDS.getY() - Y_COORDS_OFFSET, OLD_POINT3_COORDS.getZ());
    static final Point POINT5_COORDS = new Point(TARGET5_COORDS.getX() - Y_COORDS_OFFSET, TARGET5_COORDS.getY() + Z_COORDS_OFFSET, OLD_POINT5_COORDS.getZ());
    static final Point POINT6_COORDS = new Point(OLD_POINT6_COORDS.getX() + Y_COORDS_OFFSET, TARGET6_COORDS.getY() - Y_COORDS_OFFSET, TARGET6_COORDS.getZ() + Z_COORDS_OFFSET);

    static final Point COMMON_COORDS = new Point(POINT4_COORDS.getX(), POINT7_COORDS.getY(), OLD_POINT5_COORDS.getZ());

    static final List<Point> POINTS_COORDS = Arrays.asList(POINT1_COORDS, POINT2_COORDS, POINT3_COORDS,
            POINT4_COORDS, POINT5_COORDS, POINT6_COORDS, POINT7_COORDS);

    static final Quaternion START_QUATERNION = new Quaternion((float) 1, (float) 0, (float) 0, (float) 0);
    static final Quaternion GOAL_QUATERNION = new Quaternion((float) 0, (float) 0, (float) -0.707, (float) 0.707);

    static final Quaternion POINT1_QUATERNION = new Quaternion((float) 0, (float) 0, (float) -0.707, (float) 0.707);
    static final Quaternion POINT2_QUATERNION = new Quaternion((float) 0.5, (float) 0.5, (float) -0.5, (float) 0.5);
    static final Quaternion POINT3_QUATERNION = new Quaternion((float) 0, (float) 0.707, (float) 0, (float) 0.707);
    static final Quaternion POINT4_QUATERNION = new Quaternion((float) 0, (float) 0, (float) -1, (float) 0);
    static final Quaternion POINT5_QUATERNION = new Quaternion((float) -0.5, (float) -0.5, (float) -0.5, (float) 0.5);
    static final Quaternion POINT6_QUATERNION = new Quaternion((float) 0, (float) 0, (float) 0, (float) 1);
    static final Quaternion POINT7_QUATERNION = new Quaternion((float) 0, (float) 0.707, (float) 0, (float) 0.707);
    
    static final Quaternion TARGET1_QUATERNION = new Quaternion((float) 0.707, (float) 0, (float) 0, (float) 0.707);
    static final Quaternion TARGET2_QUATERNION = new Quaternion((float) 0, (float) 0, (float) 0, (float) 1);
    static final Quaternion TARGET3_QUATERNION = new Quaternion((float) 0.707, (float) 0, (float) 0, (float) 0.707);
    static final Quaternion TARGET4_QUATERNION = new Quaternion((float) -0.5, (float) 0.5, (float) -0.5, (float) 0.5);
    static final Quaternion TARGET5_QUATERNION = new Quaternion((float) 1, (float) 0, (float) 0, (float) 0);
    static final Quaternion TARGET6_QUATERNION = new Quaternion((float) 0.5, (float) 0.5, (float) -0.5, (float) -0.5);
    static final Quaternion QR_CODE_QUATERNION = new Quaternion((float) 0, (float) 0, (float) 0, (float) 1);

    static final List<Quaternion> POINTS_QUATERNIONS = Arrays.asList(POINT1_QUATERNION, POINT2_QUATERNION, POINT3_QUATERNION,
            POINT4_QUATERNION, POINT5_QUATERNION, POINT6_QUATERNION, POINT7_QUATERNION);
    
    private PointConstants() {
        
    }
}