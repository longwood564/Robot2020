package frc.robot;

/**
 * This class has constants used in other parts of the robot code. These
 * include: physics constants, field measurements, robot measurements, and
 * constant calculations.
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
   * The distance from the floor to the outer hexagon of the power port.
   */
  private static final double kVertDistanceFloorToHex = 8;
  /**
   * The distance from the floor to the inner hoop of the power port.
   * 
   * @todo Find this measurement.
   */
  private static final double kVertDistanceFloorToHoop = 8.5;
  /**
   * The distance from the outer hexagon of the power cell to the inner hoop.
   * 
   * @todo Find this measurement.
   */
  public static final double kHorDistanceHexagonToHoop = 1;

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
   * 
   * @todo Find this measurement.
   */
  public static final double kLauncherAngle = Math.toRadians(45);
  /**
   * The initial velocity of the ball as it is launched.
   * 
   * @todo Find this measurement.
   */
  public static final double kInitialVelocityBall = 10;

  // Constant Calculations

  /**
   * The projected horizontal distance from the launcher to the top of the
   * projectile motion. See the research document for the derivation of this
   * equation.
   */
  public static final double kProjectedHorDistanceToApex = (Math.pow(kInitialVelocityBall, 2) * Math.sin(kLauncherAngle)
      * Math.cos(kLauncherAngle)) / kAccelDueToGravity;
}