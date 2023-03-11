package frc.robot.math2;

public class Matrix {
   private double[][] matrix;
   private int rows;
   private int columns;
   
   public Matrix(int rows, int columns) {
        matrix = new double[rows][columns];
        this.rows = rows;
        this.columns = columns;
   }

   public Matrix matrixMultiply(Matrix m) {
       Matrix newMatrix = new Matrix(this.rows, m.columns);
       if(this.columns == m.rows) {
            for(int r = 0; r < this.rows; r++) {
                for(int c = 0; c < m.columns; c++) {
                    newMatrix.matrix[r][c] = retreiveSum(this.matrix[r], m, c);
                }

            }
       }
       return newMatrix; 
   }

   public void setItem(double number, int row, int column) {
        matrix[row][column] = number;
   }

   public double getItem(int r, int c) {
        return matrix[r][c];
   }

   public double retreiveSum(double[] one, Matrix two, int col) {
        double sum = 0;
            for(int oneRow = 0; oneRow < one.length; oneRow++) {
                sum += one[oneRow] * two.matrix[oneRow][col];
            }
        return sum;
    }

   @Override
   public String toString() {
        String e = "";
        for(int r = 0; r < matrix.length; r++) {
            e += "\n";
            for(int c = 0; c < matrix[r].length; c++) {
                e += matrix[r][c] + " ";
            }
        }
        return e;
   }
}
