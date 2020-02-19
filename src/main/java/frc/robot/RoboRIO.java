package frc.robot;

/**
 * This class has constants used in other parts of the robot code. Particularly,
 * these constants directly relate to hardware connected to the roboRIO
 */
public final class RoboRIO {
  // CAN Devices
  /**
   * The port of the left launcher motor.
   */
  public static final int kPortMotorLeftLauncher = 8;
  /**
   * The port of the right launcher motor. On the electronics board, this is #8b,
   * but this couldn't be replicated due to software limitaitons.
   */
  public static final int kPortMotorRightLauncher = 11;

  // Analog Inputs
  /**
   * The port of the ultrasonic sensor.
   */
  public static final int kPortUltrasonicSensorPort = 0;
}