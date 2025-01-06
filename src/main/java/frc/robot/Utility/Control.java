package frc.robot.Utility;

public class Control {

    /**
     * Linearly interpolates a value in a double array where the first column
     * matches value and the second matches the return value
     * 
     * @param value value to interpolate
     * @param array array of points ususally read from a csv file
     * @return Interpolated output
     */
    public static double interpolateCSV(double value, double[][] array) {
        double[] x = ArrayHelpers.getColumn(array, 0);
        double[] y = ArrayHelpers.getColumn(array, 1);
        int index = -1;
        for (int i = 1; i<=x.length - 1; i++) {
            index += 1;
            if (value < x[i]) {
                break;
            }
        }
        
        return y[index] + (value - x[index]) * (y[index+1] - y[index]) / (x[index+1] - x[index]);
    }

    /**
     * Forces a value between a minimum and a maximum such that min <= input <= max
     * 
     * @param input the value to restrict
     * @param min   the lower bound
     * @param max   the upper bound
     * @return The input restricted between the min and max
     */
    public static double clamp(double input, double min, double max) {
        return Math.max(min, Math.min(max, input));
    }
}
