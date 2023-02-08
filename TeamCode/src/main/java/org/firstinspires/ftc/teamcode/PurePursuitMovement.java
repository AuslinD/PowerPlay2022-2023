package org.firstinspires.ftc.teamcode;

public class PurePursuitMovement extends PurePursuitMath {
    public static void goToPosition(double x, double y, double moveSpeed){
        double RobotXPosition, RobotYPosition;
        RobotXPosition = 0;
        RobotYPosition = 0;
        double RobotOrientation = 0;

        double distanceToTarget = Math.hypot(x - RobotXPosition, y - RobotYPosition);
        double absoluteAngleToTarget = Math.atan2(y - RobotYPosition, x - RobotXPosition);
        //90 is forward
        double relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - (RobotOrientation - Math.toRadians(90)));

        //x and y components
        double relativeXToTarget = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToTarget = Math.sin(relativeAngleToTarget) * distanceToTarget;

        double XPower = relativeXToTarget / (Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget));
        double YPower = relativeYToTarget / (Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget));

        //implement
        //movement_x = XPower * moveSpeed;
        //movement_y = YPower * moveSpeed;
    }
}
