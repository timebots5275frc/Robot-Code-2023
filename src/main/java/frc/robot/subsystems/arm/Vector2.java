package frc.robot.subsystems.arm;

import javax.swing.text.View;

public class Vector2{
    public double x;
    public double y;

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
        Vector2 relativePos = a.substract(b);

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
}
