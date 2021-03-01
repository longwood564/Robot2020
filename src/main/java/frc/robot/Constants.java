package frc.robot;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;

/**
 * This class provides general constants relating to the laws of physics, the measurements of the
 * field, and the measurements of the robot.
 * 
 * All values are in, unless specified otherwise: - m, for distance. - m/s^2, for acceleration. -
 * rad, for angles.
 */
public final class Constants {
  // Physical Constants

  /** The amount of inches in one meter. */
  public static final double kInchesPerMeter = 39.37;
  /**
   * Acceleration due to gravity on Earth. This acceleration is positive here, it must be interpreted
   * as negative or positive when it is plugged into an equation.
   */
  public static final double kAccelDueToGravity = 9.80665;

  // Field Measurements

  /**
   * The distance from the floor to the bottom of the outer hexagon of the power port. Field
   * measurements used:
   * <ul>
   * <li>Height of hexagon = 24in.
   * <li>Half height of hexagon = 24in. / 2 = 12in.
   * <li>Floor to center of hexagon = 98.25in.
   * <li>Floor to bottom of hexagon = 98.25in. - 12in. = 86.25 in.
   * </ul>
   */
  private static final double kVertDistanceFloorToHex = 86.25 / kInchesPerMeter;
  /**
   * The distance from the floor to the inner hoop of the power port. Field measurements used:
   * <ul>
   * <li>Floor to center of hoop = 98.25in.
   * </ul>
   */
  private static final double kVertDistanceFloorToHoop =
      98.25 / kInchesPerMeter;
  /**
   * The distance from the outer hexagon of the power cell to the inner hoop. Field measurements used:
   * <ul>
   * <li>Hexagon to hoop = 29.25 in.
   * </ul>
   */
  public static final double kHorDistanceHexagonToHoop =
      29.25 / kInchesPerMeter;
  /**
   * The width of the hexagon and reflective tape. Field measurements used:
   * <ul>
   * <li>Hexagon width = 39.25 in.
   * </ul>
   */
  public static final double kWidthHexagon = 39.25 / kInchesPerMeter;

  // Robot Measurements

  /** The distance from the floor to the point at which the ball is launched. */
  // TODO: Verify this measurement.
  private static final double kVertDistanceFloorToLauncher = 0.3;
  /**
   * The distance from the point at which the ball is launched to the outer hexagon.
   */
  public static final double kVertDistanceLauncherToHex =
      kVertDistanceFloorToHex - kVertDistanceFloorToLauncher;
  /**
   * The distance from the point at which the ball is launched to the inner hoop.
   */
  public static final double kVertDistanceLauncherToHoop =
      kVertDistanceFloorToHoop - kVertDistanceFloorToLauncher;
  /** The angle of the ball launcher. */
  // TODO: Verify this measurement.
  public static final double kLauncherAngle = Math.toRadians(37);
  /** The initial velocity of the ball as it is launched. */
  // TODO: Find this measurement.
  public static final double kInitialVelocityBall = 10;

  // Configured Constants

  /** The number to multiply the joystick input by for driving at a slow speed. */
  public static final double kMultiplierSlowSpeed = 0.5;
  /**
   * The number to multiply the joystick input by for driving at a normal speed.
   */
  public static final double kMultiplierNormalSpeed = 0.65;
  /** The number to multiply the joystick input by for driving at a high speed. */
  public static final double kMultiplierHighSpeed = 0.75;
  /** The speed to set the control panel motor to. */
  // TODO: Tune this value.
  public static final double kSpeedControlPanel = 0.25;
  /** The speed to set the intake motor to. */
  // TODO: Tune this value.
  public static final double kSpeedIntake = 0.75;
  /** The speed to set the intake motor to. */
  public static final double kSpeedLauncher = 1.0;
  /** The speed to set the belt motor to. */
  // TODO: Tune this value.
  public static final double kSpeedBelt = 0.50;
  /**
   * The proportionality constant to use for correcting error in maintaining a distance from an
   * object.
   */
  // TODO: Tune this measurement.
  public static final double kP = 0.3;
  /** Color constants for vision */
  public static final Scalar kColorRed = new Scalar(255, 0, 0);
  public static final Scalar kColorGreen = new Scalar(0, 255, 0);
  public static final Scalar kColorBlue = new Scalar(0, 0, 255);

  /** The area of the camera feed which is actually used by vision processing */
  public static final Rect scanArea = new Rect(0, 120, 640, 120);

  /**
   * The focal length of the camera, determined experimentally. This is used to determine distance
   * with the camera.
   */
  public static final double kCameraFocal = 699.516;

  // Constant Calculations

  /**
   * The projected horizontal distance from the launcher to the top of the projectile motion. See the
   * research document for the derivation of this equation.
   */
  public static final double kProjectedHorDistanceToApex =
      (Math.pow(kInitialVelocityBall, 2) * Math.sin(kLauncherAngle)
          * Math.cos(kLauncherAngle)) / kAccelDueToGravity;
}
