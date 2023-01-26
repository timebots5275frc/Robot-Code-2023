package frc.robot.subsystems.arm;

import frc.robot.constants.Constants;

public class TwoJointInverseKinematics {
    private double firstJointLength;
    private double secondJointLength;

    public TwoJointInverseKinematics(double firstLength, double secondLength)
    {
        firstJointLength = firstLength;
        secondJointLength = secondLength;
    }

    public double solveFirstJoint(double targetX, double targetY)
    {
        double distance = distance(0, 0, targetX, targetY);
        boolean negative = lawOfCosines(firstJointLength, distance, secondJointLength) < 0;

        return (Math.atan(targetY/targetX) - lawOfCosines(firstJointLength, distance, secondJointLength) * (negative ? 1 : -1)) * Constants.ArmConstants.RAD_TO_DEG_RATIO;
    }

    public double solveSecondJoint(double targetX, double targetY)
    {
        double distance = distance(0, 0, targetX, targetY);
        return lawOfCosines(firstJointLength, secondJointLength, distance) * Constants.ArmConstants.RAD_TO_DEG_RATIO;
    }

    public double lawOfCosines(double leg1, double leg2, double hypotenuse)
    {
        return Math.acos((Math.pow(leg1, 2) + Math.pow(leg2, 2) - Math.pow(hypotenuse, 2)) / (2 * leg1 * leg2));
    }

    public double distance(double x1, double y1, double x2, double y2)
    {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public double[] normalizeVector(double x, double y)
    {
        double distance = distance(0, 0, x, y);
        return new double[] {x / distance, y / distance};
    }

    public double totalDistance() { return firstJointLength + secondJointLength; }
}