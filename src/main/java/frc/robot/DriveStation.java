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
  public static final int kIDButtonA = 1;
  /** The button ID of the B button. */
  public static final int kIDButtonB = 2;
  /** The button ID of the X button. */
  public static final int kIDButtonX = 3;
  /** The button ID of the Y button. */
  public static final int kIDButtonY = 4;
  /** The button ID of LB, the right bumper. */
  public static final int kIDButtonLB = 5;
  /** The button ID of RB, the right bumper. */
  public static final int kIDButtonRB = 6;
  /** The button ID of the back button, also select on some controllers. */
  public static final int kIDButtonBack = 7;
  /** The button ID of the start button. */
  public static final int kIDButtonStart = 8;
  /** The button ID of LS, activated by clicking the left stick. */
  public static final int kIDButtonLS = 9;
  /** The button ID of RS, activated by clicking the right stick. */
  public static final int kIDButtonRS = 10;

  // Axis IDs

  /** The axis ID of the X dimension of the left analog stick. */
  public static final int kIDAxisLeftX = 0;
  /** The axis ID of the Y dimension of the left analog stick. */
  public static final int kIDAxisLeftY = 1;
  /** The axis ID of LT, the left trigger. */
  public static final int kIDAxisLT = 2;
  /** The axis ID of RT, the right trigger. */
  public static final int kIDAxisRT = 3;
  /** The axis ID of the X dimension of the right analog stick. */
  public static final int kIDAxisRightX = 4;
  /** The axis ID of the Y dimension of the right analog stick. */
  public static final int kIDAxisRightY = 5;
}
