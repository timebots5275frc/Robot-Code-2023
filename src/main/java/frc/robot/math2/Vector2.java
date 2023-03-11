package frc.robot.math2;

import javax.swing.text.View;

import frc.robot.constants.Constants.ArmConstants;

public class Vector2{
    public double x;
    public double y;

    public static Vector2 zero = new Vector2(0, 0);

    public Vector2(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public double magnitude()
    {
        return Vector2.distance(x, y, 0, 0);
    }

    public static double distance(double x1, double y1, double x2, double y2)
    {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public static double distance(Vector2 a, Vector2 b)
    {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    public Vector2 normalized()
    {
        double distance = distance(0, 0, x, y);
        return this.divideBy(distance);
    }

    public static Vector2 lerp(Vector2 a, Vector2 b, double percent)
    {
        Vector2 relativePos = b.substract(a);

        return a.add(relativePos.times(percent));
    }

    public static Vector2 clampMagnitude(Vector2 vector2, double max)
    {
        Vector2 out = vector2;
        if (vector2.magnitude() > max) { out = vector2.normalized().times((max));}

        return out;
    }

    public Vector2 times(double b)
    {
        return new Vector2(x * b, y * b);
    }

    public Vector2 add(Vector2 b)
    {
        return new Vector2(x + b.x, y + b.y);
    }

    public Vector2 substract(Vector2 a)
    {
        return new Vector2(x - a.x, y - a.y);
    }

    public Vector2 divideBy(double b)
    {
        return new Vector2(x / b, y / b);
    }
    
    public Vector2 RotateVectorByDegrees(Vector2 orig, double degrees) { return RadToVector2(Vector2ToRad(orig) + (degrees * ArmConstants.DEG_TO_RAD_RATIO)).times(orig.magnitude()); }

    public static double Vector2ToRad(Vector2 orig)
    {
        double angle;
        if (orig.x == 0) 
        {
            if (orig.y < 0) { angle = 270 * ArmConstants.DEG_TO_RAD_RATIO; }
            else { angle = 90 * ArmConstants.DEG_TO_RAD_RATIO; }
        }
        
        else if (orig.y == 0)
        {
            if (orig.x < 0) { angle = 180 * ArmConstants.DEG_TO_RAD_RATIO; }
            else { angle = 0 * ArmConstants.DEG_TO_RAD_RATIO; }
        }

        else { angle = Math.atan(orig.y / orig.x); if (orig.x < 0) { angle += Math.PI; } }

        return angle;
    }

    public static Vector2 RadToVector2(double angle)
    {
        angle %= 360;
        double cos = Math.cos(angle);
        return new Vector2(cos * (angle > 90 && angle < 270 ? -1 : 1), Math.sin(angle));
    }
}
