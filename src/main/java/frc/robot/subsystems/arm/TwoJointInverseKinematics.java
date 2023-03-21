package frc.robot.subsystems.arm;

import frc.robot.constants.Constants;
import frc.robot.math2.Vector2;

public class TwoJointInverseKinematics {
    private double firstJointLength;
    private double secondJointLength;

    public TwoJointInverseKinematics(double firstLength, double secondLength)
    {
        firstJointLength = firstLength;
        secondJointLength = secondLength;
    }

    public double solveFirstJoint(Vector2 target)
    {
        double firstAngle;
        double distance = Vector2.distance(Vector2.zero, target);
        if (target.x >= 0) {
            firstAngle = -((Math.atan(target.y/target.x) - lawOfCosines(firstJointLength, distance, secondJointLength)) * 180/Math.PI);
        } else {
            firstAngle = -( 180 + (Math.atan(target.y/target.x) - lawOfCosines(firstJointLength, distance, secondJointLength)) * 180/Math.PI);
            if (firstAngle > 0) {
                return firstAngle;
            } else {
                return -firstAngle + ((180 + firstAngle) * 2);
            }
        }
        return firstAngle;
    }

    public double solveSecondJoint(Vector2 target)
    {
        double distance = Vector2.distance(Vector2.zero, target);
        return 180 - lawOfCosines(firstJointLength, secondJointLength, distance) * Constants.ArmConstants.RAD_TO_DEG_RATIO;
    }

    public double lawOfCosines(double leg1, double leg2, double hypotenuse)
    {
        return Math.acos((Math.pow(leg1, 2) + Math.pow(leg2, 2) - Math.pow(hypotenuse, 2)) / (2 * leg1 * leg2));
    }

    public double totalDistance() { return firstJointLength + secondJointLength; }
}