package frc.robot.Utility.ClassHelpers;

public class DriverProfile {
  public String profile;
  public double strafeDeadband;
  public double strafeScaling;
  public double strafeMax;
  public double rotateDeadband;
  public double rotateScaling;
  public double rotateMax;

  /**
   * Used for storing driver profile values in an organized way
   * 
   * @param profile        Name of the driver
   * @param strafeDeadband minimum left joystick value to output anything
   * @param strafeScaling  how quickly the left joystick output increases past the
   *                       deadband
   * @param strafeMax      the highest left joystick output allowed
   * @param rotateDeadband minimum right joystick value to output anything
   * @param rotateScaling  how quickly the right joystick output increases past
   *                       the deadband
   * @param rotateMax      the highest right joystick output allowed
   */
  public DriverProfile(String profile, double strafeDeadband, double strafeScaling, double strafeMax,
      double rotateDeadband, double rotateScaling, double rotateMax) {
    this.profile = profile;
    this.strafeDeadband = strafeDeadband;
    this.strafeScaling = strafeScaling;
    this.strafeMax = strafeMax;
    this.rotateDeadband = rotateDeadband;
    this.rotateScaling = rotateScaling;
    this.rotateMax = rotateMax;
  }

  /**
   * Replaces the current stored DriverProfile with new values
   * 
   * @param profile        Name of the driver
   * @param strafeDeadband minimum left joystick value to output anything
   * @param strafeScaling  how quickly the left joystick output increases past the
   *                       deadband
   * @param strafeMax      the highest left joystick output allowed
   * @param rotateDeadband minimum right joystick value to output anything
   * @param rotateScaling  how quickly the right joystick output increases past
   *                       the deadband
   * @param rotateMax      the highest right joystick output allowed
   */
  public DriverProfile update(String profile, double strafeDeadband, double strafeScaling, double strafeMax,
      double rotateDeadband, double rotateScaling, double rotateMax) {
    this.profile = profile;
    this.strafeDeadband = strafeDeadband;
    this.strafeScaling = strafeScaling;
    this.strafeMax = strafeMax;
    this.rotateDeadband = rotateDeadband;
    this.rotateScaling = rotateScaling;
    this.rotateMax = rotateMax;
    return this;
  }

  public double[] toDoubleArray() {
    return new double[] { strafeDeadband, strafeScaling, strafeMax, rotateDeadband, rotateScaling, rotateMax };
  }
}
