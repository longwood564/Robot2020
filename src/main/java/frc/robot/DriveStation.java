package frc.robot;

/**
 * This class provides constants relating to the hardware connected to the drive station laptop.
 */
public final class DriveStation {
  // USB Devices

  /** The port of the driver controller. */
  public static final int kPortControllerDrive = 0;
  /** The port of the manipulator controller. */
  public static final int kPortControllerManip = 1;

  // Button IDs

  /** The button ID of the A button. */
  public static final int kIdButtonA = 1;
  /** The button ID of the B button. */
  public static final int kIdButtonB = 2;
  /** The button ID of the X button. */
  public static final int kIdButtonX = 3;
  /** The button ID of the Y button. */
  public static final int kIdButtonY = 4;
  /** The button ID of LB, the right bumper. */
  public static final int kIdButtonLb = 5;
  /** The button ID of RB, the right bumper. */
  public static final int kIdButtonRb = 6;
  /** The button ID of the back button, also select on some controllers. */
  public static final int kIdButtonBack = 7;
  /** The button ID of the start button. */
  public static final int kIdButtonStart = 8;
  /** The button ID of LS, activated by clicking the left stick. */
  public static final int kIdButtonLs = 9;
  /** The button ID of RS, activated by clicking the right stick. */
  public static final int kIdButtonRs = 10;

  // POV IDs

  /** The POV ID of the D-pad. */
  public static final int kIdPovDpad = 0;

  // Axis IDs

  /** The axis ID of the X dimension of the left analog stick. */
  public static final int kIdAxisLeftX = 0;
  /** The axis ID of the Y dimension of the left analog stick. */
  public static final int kIdAxisLeftY = 1;
  /** The axis ID of LT, the left trigger. */
  public static final int kIdAxisLt = 2;
  /** The axis ID of RT, the right trigger. */
  public static final int kIdAxisRt = 3;
  /** The axis ID of the X dimension of the right analog stick. */
  public static final int kIdAxisRightX = 4;
  /** The axis ID of the Y dimension of the right analog stick. */
  public static final int kIdAxisRightY = 5;
}
