package frc.robot.subsystems.arm;

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
        return lawOfCosines(firstJointLength, distance, secondJointLength);
    }

    public double solveSecondJoint(double targetX, double targetY)
    {
        double distance = distance(0, 0, targetX, targetY);
        return lawOfCosines(firstJointLength, secondJointLength, distance);
    }

    public double lawOfCosines(double leg1, double leg2, double hypotenuse)
    {
        return Math.acos((Math.pow(leg1, 2) + Math.pow(leg2, 2) - Math.pow(hypotenuse, 2)) / (2 * leg1 * leg2));
    }

    public double distance(double x1, double y1, double x2, double y2)
    {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
}