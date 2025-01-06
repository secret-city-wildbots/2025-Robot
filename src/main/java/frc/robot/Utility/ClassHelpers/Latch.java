package frc.robot.Utility.ClassHelpers;

public class Latch {
    boolean defaultBool;
    boolean latchedBool;

    /**
     * Constantly updates a boolean value and stores it upon recieving a latch
     * signal and stops storing it upon recieving a reset signal
     * 
     * @param defaultValue True or False
     */
    public Latch(boolean defaultValue) {
        this.defaultBool = defaultValue;
        latchedBool = defaultValue;
    }

    public boolean updateLatch(boolean latchValue, boolean defaultValue, boolean latchSignal, boolean reset) {
        latchedBool = (reset) ? defaultValue : (latchSignal) ? latchValue : latchedBool;
        return latchedBool;
    }

    double defaultDbl;
    double latchedDbl;

    /**
     * Constantly updates a double value and stores it upon recieving a latch signal
     * and stops storing it upon recieving a reset signal
     * 
     * @param defaultValue Double
     */
    public Latch(double defaultValue) {
        this.defaultDbl = defaultValue;
        latchedDbl = defaultValue;
    }

    public double updateLatch(double latchValue, double defaultValue, boolean latchSignal, boolean reset) {
        latchedDbl = (reset) ? defaultValue : (latchSignal) ? latchValue : latchedDbl;
        return latchedDbl;
    }
}
