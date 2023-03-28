package frc.robot.subsystems.arm;
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

        firstAngle = -((Math.atan2(target.y, target.x) - lawOfCosines(firstJointLength, distance, secondJointLength)) * 180/Math.PI);

        return firstAngle;
    }

    public double solveSecondJoint(Vector2 target)
    {
        double distance = Vector2.distance(Vector2.zero, target);
        return 180 - lawOfCosines(firstJointLength, secondJointLength, distance) * 180/Math.PI;
    }

    public double lawOfCosines(double leg1, double leg2, double leg3)
    {
        return Math.acos((Math.pow(leg1, 2) + Math.pow(leg2, 2) - Math.pow(leg3, 2)) / (2 * leg1 * leg2));
    }

    public double totalDistance() { return firstJointLength + secondJointLength; }
}