package frc.robot.Utility;

public class LEDHelpers {

    /**
     * Converts hsv value inputs to RGB
     * @param hue
     * @param saturation
     * @param value
     * @return A string containing the rgb values
     */
    public static String hsvToRgb(float hue, float saturation, float value) {

        int h = (int)(hue * 6) / 360;
        float f = hue * 6 - h;
        float p = value * (1 - saturation);
        float q = value * (1 - f * saturation);
        float t = value * (1 - (1 - f) * saturation);
    
        switch (h) {
          case 0: return rgbToString(value, t, p);
          case 1: return rgbToString(q, value, p);
          case 2: return rgbToString(p, value, t);
          case 3: return rgbToString(p, q, value);
          case 4: return rgbToString(t, p, value);
          case 5: return rgbToString(value, p, q);
          default: throw new RuntimeException("Something went wrong when converting from HSV to RGB. Input was " + hue + ", " + saturation + ", " + value);
        }
    }



    /**
     * Converts three rgb inputs into a String
     * @param r
     * @param g
     * @param b
     * @return A string combining r, g, and b
     */
    public static String rgbToString(float r, float g, float b) {
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
     * @param rgb RGB input
     * @return GRB ouput
     */
    public static String rgbtogrb(String rgb) {
        return rgb.substring(3, 6) + rgb.substring(0, 3) + rgb.substring(6, 9);
    }
}
