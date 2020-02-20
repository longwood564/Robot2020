package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class handles robot initialization with WPILib.
 */
public final class Main {
  private Main() {
  }

  /**
   * Starts the robot code.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
