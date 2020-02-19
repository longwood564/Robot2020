package frc.robot;

/**
 * This class has constants used in other parts of the robot code. These
 * include: physics constants, field measurements, robot ports, robot
 * measurements, and constant calculations.
 * 
 * All values are in, unless specified otherwise: - m, for distance. - m/s^2,
 * for acceleration. - rad, for angles.
 */
public final class Constants {
  // Physics Constants

  /**
   * Acceleration due to gravity on Earth. This acceleration is positive here, it
   * must be interpreted as negative or positive when it is plugged into an
   * equation.
   */
  public static final double kAccelDueToGravity = 9.80665;

  // Field Measurements

  /**
   * The distance from the floor to the bottom of the outer hexagon of the power
   * port. Field measurements used:
   * <ul>
   * <li>Height of hexagon = 24in.
   * <li>Half height of hexagon = 24in. / 2 = 12in.
   * <li>Floor to center of hexagon = 98.25in.
   * <li>Floor to bottom of hexagon = 98.25in. - 12in. = 86.25 in.
   * </ul>
   */
  private static final double kVertDistanceFloorToHex = 86.25 / 39.37;
  /**
   * The distance from the floor to the inner hoop of the power port. Field
   * measurements used:
   * <ul>
   * <li>Floor to center of hoop = 98.25in.
   * </ul>
   */
  private static final double kVertDistanceFloorToHoop = 98.25 / 39.37;
  /**
   * The distance from the outer hexagon of the power cell to the inner hoop.
   * Field measurements used:
   * <ul>
   * <li>Hexagon to hoop = 29.25 in.
   * </ul>
   */
  public static final double kHorDistanceHexagonToHoop = 29.25 / 39.37;

  // Robot Measurements

  /**
   * The distance from the floor to the point at which the ball is launched.
   */
  private static final double kVertDistanceFloorToLauncher = 0.3;
  /**
   * The distance from the point at which the ball is launched to the outer
   * hexagon.
   */
  public static final double kVertDistanceLauncherToHex = kVertDistanceFloorToHex - kVertDistanceFloorToLauncher;
  /**
   * The distance from the point at which the ball is launched to the inner hoop.
   */
  public static final double kVertDistanceLauncherToHoop = kVertDistanceFloorToHoop - kVertDistanceFloorToLauncher;
  /**
   * The angle of the ball launcher.
   */
  public static final double kLauncherAngle = Math.toRadians(37);
  /**
   * The initial velocity of the ball as it is launched.
   * 
   * @todo Find this measurement.
   */
  public static final double kInitialVelocityBall = 10;
  /**
   * The proportionality constant to use for correcting error in maintaining a
   * distance from an object.
   */
  public static final double kP = 0.05;
  /**
   * The conversion from the voltage reading of the analog ultrasonic sensor, to
   * meters. The voltage / distance range for the MB1013 is 300-mm / 293mV,
   * 5000-mm / 4.885V. For more info, see the description of pin 3 of the board
   * here: https://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf
   */
  public static final double kMetersPerVolt = (5000 / 4.885) * (1.0 / 1000);
  /**
   * The minimum distance that the distance sensor can read.
   * 
   * @see #kMetersPerVolt
   */
  public static final double kMinimumUltrasonicReading = (300) * (1.0 / 1000);
  /**
   * The maximum distance that the distance sensor can read. See above for info on
   * where this value came from.
   * 
   * @see #kMetersPerVolt
   */
  public static final double kMaximumUltrasonicReading = (5000) * (1.0 / 1000);
  /**
   * The range of values that the ultrasonic sensor will read.
   * 
   * @see #kMetersPerVolt
   */
  public static final double kUltrasonicRange = kMaximumUltrasonicReading - kMinimumUltrasonicReading;

  // Constant Calculations

  /**
   * The projected horizontal distance from the launcher to the top of the
   * projectile motion. See the research document for the derivation of this
   * equation.
   */
  public static final double kProjectedHorDistanceToApex = (Math.pow(kInitialVelocityBall, 2) * Math.sin(kLauncherAngle)
      * Math.cos(kLauncherAngle)) / kAccelDueToGravity;
}