package frc.robot.Utility.ClassHelpers;

public class BlinkLight {
    private double period;
    private Timer time;

    public BlinkLight(double period) {
        this.period = period;
        this.time = new Timer();
    }

    /**
     * Returns a periodically toggled true or false where the period is period
     * 
     * @param period How frequently to switch between true and false
     * @return
     */
    public boolean blinkLight() {
        return ((time.getTimeMillis() % period) * 2) < period;
    }
}
