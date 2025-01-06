package frc.robot.Utility.ClassHelpers;

public class Timer {
    private double time0 = (double) System.currentTimeMillis();

    /**
     * @return The time since the last reset
     */
    public double getTimeMillis() {
        return (double) (System.currentTimeMillis()) - time0;
    }

    /**
     * Resets the timer
     */
    public void reset() {
        time0 = (double) System.currentTimeMillis();
    }
}
