package frc.robot.Utility;

public class ArrayHelpers {

    /**
     * Gets the column in a 2d array at array[...][index]
     * 
     * @param array The 2d array to get the column from
     * @param index The index of the column
     * @return The 1d array containing each element from the chosen column
     */
    public static double[] getColumn(double[][] array, int index) {
        double[] column = new double[array.length];
        for (int i = 0; i < array.length; i++) {
            column[i] = array[i][index];
        }
        return column;
    }
}
