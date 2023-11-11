package frc.robot.constants;

import frc.robot.math2.Vector2;

public class MovePoint {
    private Vector2 point;
    private String pointName;
    public MovePoint(Vector2 p, String pn) {
        point = p;
        pointName = pn;
    }
    public Vector2 getPoint() {
        return point;
    }
    public String getName() {
        return pointName;
    }
}
