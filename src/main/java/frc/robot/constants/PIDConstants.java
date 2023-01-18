package frc.robot.constants;

public class PIDConstants {
    public double P;
    public double I;
    public double D;
    public double Iz;
    public double kFF;

    public PIDConstants(double _P, double _I, double _D) {
        this(_P, _I, _D, 0.0, 0.0);
    }

    public PIDConstants(double p, double i, double d, double iz, double kff) {
        P = p;
        I = i;
        D = d;
        Iz = iz;
        kFF = kff;
    }
}
