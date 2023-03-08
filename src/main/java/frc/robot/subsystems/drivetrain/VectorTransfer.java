package frc.robot.subsystems.drivetrain;

import frc.robot.subsystems.arm.Vector2;

public class VectorTransfer {
    public static double[][] matrix;

    public static double angle = 0;

    public static double alpha = 0; // Yaw

    public static double beta = 0; // Pitch

    public static double gamma = 0; // Roll

    public static double[] balls = {0, 0, 1};

    public static final double[][] xMartix = {{1, 0, 0}, {0, Math.cos(angle), -Math.sin(angle)}, {0, Math.sin(angle), Math.cos(angle)}};
    public static final double[][] yMartix = {{Math.cos(angle), 0, Math.sin(angle)}, {0, 1, 0}, {-Math.sin(angle), 0, Math.cos(angle)}};
    public static final double[][] zMartix = {{Math.cos(angle), -Math.sin(angle), 0}, {Math.sin(angle), Math.cos(angle), 0}, {0, 0, 1}};

    public static final double[][] generalRotationMatrixIntrinsic = {{Math.cos(alpha) * Math.cos(beta), Math.cos(alpha) * Math.sin(beta) * Math.sin(gamma) - Math.sin(alpha) * Math.cos(gamma), Math.cos(alpha) * Math.sin(beta) * Math.cos(gamma) + Math.sin(alpha) * Math.sin(gamma)},
    {Math.sin(alpha) * Math.cos(beta), Math.sin(alpha) * Math.sin(beta) * Math.sin(gamma) + Math.cos(alpha) * Math.cos(gamma), Math.sin(alpha) * Math.sin(beta), Math.cos(gamma) - Math.cos(alpha) * Math.sin(gamma)},
    {-Math.sin(beta), Math.cos(beta) * Math.sin(gamma), Math.cos(beta) * Math.cos(gamma)}};

    public static final double[][] generalRotationMatrixExtrinsic = {{Math.cos(beta) * Math.cos(gamma), Math.sin(alpha) * Math.sin(beta) * Math.cos(gamma) - Math.cos(alpha) * Math.sin(gamma), Math.cos(alpha) * Math.sin(beta) * Math.cos(gamma) + Math.sin(alpha) * Math.sin(gamma)},
    {Math.cos(beta) * Math.sin(gamma), Math.sin(alpha) * Math.sin(beta) * Math.sin(gamma) + Math.cos(alpha) * Math.cos(gamma), Math.cos(alpha) * Math.sin(beta), Math.sin(gamma) - Math.sin(alpha) * Math.cos(gamma)},
    {-Math.sin(beta), Math.sin(alpha) * Math.cos(beta), Math.cos(alpha) * Math.cos(beta)}};


    private static void sussyBalls(double[][] matrixpoop, double[] sussu) {
        double[] poopoopeepee = new double[3];
        poopoopeepee[0] = matrixpoop[0][0] * sussu[0] + matrixpoop[0][1] * sussu[0] + matrixpoop[0][2] * sussu[0];
        poopoopeepee[1] = matrixpoop[1][0] * sussu[0] + matrixpoop[1][1] * sussu[0] + matrixpoop[1][2] * sussu[0];
        poopoopeepee[2] = matrixpoop[2][0] * sussu[0] + matrixpoop[2][1] * sussu[0] + matrixpoop[2][2] * sussu[0];
    }

    public static double[] matrixMultiply(double[][] firstMatrix, double[] secondMatrix) {
        double[] newMatrix = new double[firstMatrix.length];
        if (firstMatrix.length == secondMatrix.length) {
            for(int i = 0; i < firstMatrix.length; i++) {
                double currentRow = 0;
                for(int j = 0; j < firstMatrix[i].length; j++) {
                    currentRow += firstMatrix[i][j] * secondMatrix[j];
                }
                newMatrix[i] = currentRow;
            }
            
        }
        return newMatrix;
    }


    //REal math https://en.wikipedia.org/wiki/Rotation_matrix
}