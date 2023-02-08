package org.firstinspires.ftc.teamcode;

public class PurePursuitMath {
    //sets angle to bounds within -180 to 180, could have error with 2 * Math.PI
    public static double AngleWrap(double angle){
        if(angle < -Math.PI){
            angle = -(Math.abs(angle) % (2 * Math.PI));
        }
        if(angle > Math.PI){
            angle = Math.abs(angle) % (2 * Math.PI);
        }
        return angle;
    }
}
