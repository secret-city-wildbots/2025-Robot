package frc.robot.Utility.ClassHelpers;

public class StickyButton {
    Timer timer = new Timer();
    Latch latch = new Latch(false);

    /**
     * This will keep track of if a button has been pressed within the past
     * timeThresh milliseconds
     * 
     * @param buttonPressed Button to keep track of
     * @param timeThresh    time threshold in milliseconds
     * @param reset         whether or not to reset the latch
     * @return Stuck button value
     */
    public boolean isStuck(boolean buttonPressed, double timeThresh, boolean reset) {
        if (buttonPressed) {
            timer.reset();
        }
        latch.updateLatch(true, false, buttonPressed, reset);
        return (timer.getTimeMillis() <= timeThresh) && latch.latchedBool;
    }

    /**
     * This will keep track of if a button has been pressed within the past
     * timeThresh milliseconds
     * 
     * @param buttonPressed Button to keep track of
     * @param timeThresh    time threshold in milliseconds
     * @return Stuck button value
     */
    public boolean isStuck(boolean buttonPressed, double timeThresh) {
        if (buttonPressed) {
            timer.reset();
        }
        latch.updateLatch(true, false, buttonPressed, false);
        return (timer.getTimeMillis() <= timeThresh) && latch.latchedBool;
    }
}
