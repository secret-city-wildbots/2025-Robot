package frc.robot.Utility;

public class LEDHelpers {

    /**
     * Converts hsv value inputs to RGB. This should be used with addressable LEDs bc WPI did a bad job on the ranges
     * @param hue 0-360
     * @param saturation 0-1
     * @param value 0-1
     * @return A double[] containing the rgb values
     */
    public static double[] hsvToRgb(double hue, double saturation, double value) {

        int h = (int)(hue * 6) / 360;
        double f = hue * 6 - h;
        double p = value * (1 - saturation);
        double q = value * (1 - f * saturation);
        double t = value * (1 - (1 - f) * saturation);
    
        switch (h) {
          case 0: return new double[] {value, t, p};
          case 1: return new double[] {q, value, p};
          case 2: return new double[] {p, value, t};
          case 3: return new double[] {p, q, value};
          case 4: return new double[] {t, p, value};
          case 5: return new double[] {value, p, q};
          default: throw new RuntimeException("Something went wrong when converting from HSV to RGB. Input was " + hue + ", " + saturation + ", " + value);
        }
    }

    /**
     * Converts hsv value inputs to RGB. This should be used with addressable LEDs bc WPI did a bad job on the ranges
     * @param hsv h: 0-360, s: 0-1, v: 0-1
     * @return A double[] containing the rgb values
     */
    public static double[] hsvToRgb(double[] hsv) {

        int h = (int)(hsv[0] * 6) / 360;
        double f = hsv[0] * 6 - h;
        double p = hsv[2] * (1 - hsv[1]);
        double q = hsv[2] * (1 - f * hsv[1]);
        double t = hsv[2] * (1 - (1 - f) * hsv[1]);
    
        switch (h) {
          case 0: return new double[] {hsv[2], t, p};
          case 1: return new double[] {q, hsv[2], p};
          case 2: return new double[] {p, hsv[2], t};
          case 3: return new double[] {p, q, hsv[2]};
          case 4: return new double[] {t, p, hsv[2]};
          case 5: return new double[] {hsv[2], p, q};
          default: throw new RuntimeException("Something went wrong when converting from HSV to RGB. Input was " + hsv[0] + ", " + hsv[1] + ", " + hsv[2]);
        }
    }



    /**
     * Converts three rgb inputs into a String
     * @param r
     * @param g
     * @param b
     * @return A string combining r, g, and b
     */
    public static String rgbToString(double r, double g, double b) { //this code is disgusting
        String rs = Integer.toString((int)(r * 256));
        rs = (rs.length() < 3) ? (rs.length() < 2) ? "00"+rs:"0"+rs:rs;
        String gs = Integer.toString((int)(g * 256));
        gs = (gs.length() < 3) ? (gs.length() < 2) ? "00"+gs:"0"+gs:gs;
        String bs = Integer.toString((int)(b * 256));
        bs = (bs.length() < 3) ? (bs.length() < 2) ? "00"+bs:"0"+bs:bs;
        return rs + gs + bs;
    }



    /**
     * Converts rgb color values to grb color values for use on weird LED strips
     * @param rgb RGB input string
     * @return GRB ouput
     */
    public static String rgbtogrb(String rgb) {
        return rgb.substring(3, 6) + rgb.substring(0, 3) + rgb.substring(6, 9);
    }

    /**
     * Converts rgb color values to grb color values for use on weird LED strips
     * @param r
     * @param g
     * @param b
     * @return
     */
    public static double[] rgbtogrb(double r, double g, double b) {
        return new double[] {g,r,b};
    }

    /**
     * Converts rgb color values to grb color values for use on weird LED strips
     * @param rgb RGB input double[]
     * @return GRB ouput
     */
    public static double[] rgbtogrb(double[] rgb) {
        return new double[] {rgb[1], rgb[0], rgb[2]};
    }
}
