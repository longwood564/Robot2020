package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * Runs the robot code. The name of this class is depended upon by build.gradle.
 */
public class Robot extends TimedRobot {
  // Joysticks
  private final Joystick m_controllerDrive =
      new Joystick(DriveStation.kPortControllerDrive);
  private final Joystick m_controllerManip =
      new Joystick(DriveStation.kPortControllerManip);
  private boolean m_buttonManipPressA = false;
  private boolean m_buttonManipPressB = false;
  private boolean m_buttonManipPressX = false;
  private boolean m_buttonManipPressY = false;
  private boolean m_buttonManipPressStart = false;

  // State
  private boolean m_isInControlPanelMode = false;

  // Autonomous
  private static final String kAutoCaseDefault = "Default";
  private String m_selectedAuto;
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  // Driving
  private final WPI_TalonSRX m_motorDriveFrontLeft =
      new WPI_TalonSRX(RoboRIO.kPortMotorDriveFrontLeft);
  private final WPI_TalonSRX m_motorDriveFrontRight =
      new WPI_TalonSRX(RoboRIO.kPortMotorDriveFrontRight);
  private final WPI_VictorSPX m_motorDriveBackLeft =
      new WPI_VictorSPX(RoboRIO.kPortMotorDriveBackLeft);
  private final WPI_VictorSPX m_motorDriveBackRight =
      new WPI_VictorSPX(RoboRIO.kPortMotorDriveBackRight);
  private final Compressor m_compressor =
      new Compressor(RoboRIO.kPortCompressor);
  private final DifferentialDrive m_differentialDrive =
      new DifferentialDrive(m_motorDriveFrontLeft, m_motorDriveFrontRight);

  // Launching
  WPI_VictorSPX m_motorLauncherLeft =
      new WPI_VictorSPX(RoboRIO.kPortMotorLauncherLeft);
  WPI_VictorSPX m_motorLauncherRight =
      new WPI_VictorSPX(RoboRIO.kPortMotorLauncherRight);
  private final AnalogInput m_analogInputUltrasonicSensor =
      new AnalogInput(RoboRIO.kPortUltrasonicSensorPort);
  // Leave this uninitialized because we have to configure the analog input.
  private AnalogPotentiometer m_ultrasonicSensor;

  // Control Panel
  private final ColorSensorV3 m_colorSensor =
      new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private static final Color kBlueTarget =
      ColorMatch.makeColor(0.143, 0.427, 0.429);
  private static final Color kGreenTarget =
      ColorMatch.makeColor(0.197, 0.561, 0.240);
  private static final Color kRedTarget =
      ColorMatch.makeColor(0.561, 0.232, 0.114);
  private static final Color kYellowTarget =
      ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final WPI_TalonSRX m_motorControlPanel =
      new WPI_TalonSRX(RoboRIO.kPortMotorControlPanel);
  private String m_detectedColorString = "N/A";
  private String m_lastDetectedColorString = "N/A";
  private String m_targetControlPanelColor = "N/A";
  private int m_controlPanelSpinAmount = 0;

  // Vision

  /**
   * Initializes the robot code when the robot power is turned on.
   */
  @Override
  public void robotInit() {
    // Set the PCM in closed loop control mode to enable it.
    m_compressor.setClosedLoopControl(true);

    // Slave follows master
    m_motorDriveBackRight.follow(m_motorDriveFrontRight);
    m_motorDriveBackLeft.follow(m_motorDriveFrontLeft);

    m_motorLauncherRight.follow(m_motorLauncherLeft);
    // Invert one of the launching motors, because they must spin in opposite
    // directions.
    m_motorLauncherRight.setInverted(true);

    // Configure the ultrasonic sensor.
    // Enable 2-bit averaging, for stability,
    m_analogInputUltrasonicSensor.setAverageBits(2);
    // Initialize an analog potentiometer, configured for the ultrasonic sensor.
    // The documentation for this function describes this parameter as a "scale",
    // although it is not the scale for how many units a volt represent - rather, it
    // expects the units per 5 volts.
    m_ultrasonicSensor = new AnalogPotentiometer(m_analogInputUltrasonicSensor,
        RoboRIO.kMetersPerVoltUltrasonic * 5);

    // Add color sensor matches.
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    // Add Shuffleboard sendables. We define the NetworkTableEntry objects as member
    // variables when adding those widgets because we need to access them to update
    // them. Contrary, we don't assign these widgets to any variables because, as
    // Sendable interfaces, they will automatically be updated. There is a
    // distinction to be made between assigning the ComplexWidget to a variable, and
    // assigning the SendableChooser to a variable - which we *do* do.
    m_autoChooser.setDefaultOption("Default Auto", kAutoCaseDefault);
    ShuffleboardClass.m_layoutAutonomous.add(m_autoChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);
    ShuffleboardClass.m_layoutDriving.add(m_differentialDrive);
    ShuffleboardClass.m_layoutLaunching
        .add("Optimal Distance to Apex",
            Constants.kProjectedHorDistanceToApex
                - Constants.kHorDistanceHexagonToHoop)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(ShuffleboardClass.kPropertiesDistanceSensor).getEntry();
  }

  /**
   * Updates diagnostics while the robot power is on.
   */
  @Override
  public void robotPeriodic() {
    ShuffleboardClass.shuffleboardPeriodic();
  }

  /**
   * Initializes the robot code when the simulation is started.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * Updates diagnostics while the simulation is running.
   */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * Initializes disabled mode.
   */
  @Override
  public void disabledInit() {
    ShuffleboardClass.m_entryDetectedColor.setString("N/A");
    ShuffleboardClass.m_entryConfidence.setDouble(0);
    ShuffleboardClass.m_entryTargetColor.setString("N/A");
    ShuffleboardClass.m_entryTargetSpin.setDouble(0);
  }

  /**
   * Maintains disabled mode.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * Initializes autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_selectedAuto = m_autoChooser.getSelected();

    m_compressor.start();
  }

  /**
   * Maintains autonomous mode.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_selectedAuto) {
      case kAutoCaseDefault:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * Initializes teleoperated mode.
   */
  @Override
  public void teleopInit() {
    m_isInControlPanelMode = false;

    m_detectedColorString = "N/A";
    m_lastDetectedColorString = "N/A";
    m_targetControlPanelColor = "N/A";
    m_controlPanelSpinAmount = 0;

    m_compressor.start();
  }

  /**
   * Maintains teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    updateInputs();
    handleState();
    driveSpeed();
    launchBall();
    spinControlPanel();
  }

  /**
   * Reads pressed states from the gamepads. This is done here because it is paramount that
   * getRawButtonPressed() is only called once per loop, because, the second time, it will more than
   * likely just return "false" for any button.
   */
  private void updateInputs() {
    m_buttonManipPressA =
        m_controllerManip.getRawButtonPressed(DriveStation.kIDButtonA);
    m_buttonManipPressB =
        m_controllerManip.getRawButtonPressed(DriveStation.kIDButtonB);
    m_buttonManipPressX =
        m_controllerManip.getRawButtonPressed(DriveStation.kIDButtonX);
    m_buttonManipPressY =
        m_controllerManip.getRawButtonPressed(DriveStation.kIDButtonY);
    m_buttonManipPressStart =
        m_controllerManip.getRawButtonPressed(DriveStation.kIDButtonStart);
  }

  /**
   * Handles general state of the teleoperated mode.
   */
  private void handleState() {
    if (m_buttonManipPressStart) {
      m_isInControlPanelMode = !m_isInControlPanelMode;
      ShuffleboardClass.m_entryControlPanelMode
          .setBoolean(m_isInControlPanelMode);
    } else {
      m_isInControlPanelMode = ShuffleboardClass.m_entryControlPanelMode
          .getBoolean(m_isInControlPanelMode);
    }
  }

  /**
   * Drives the robot at a certain speed inputted by the driver.
   */
  private void driveSpeed() {
    if (m_isInControlPanelMode) {
      // Explicitly stop the motors since we are in control panel mode, and do not
      // need to be moving. This is necessary because the motor power must be updated
      // for every iteration of the loop.
      m_differentialDrive.stopMotor();
    } else {
      // Left thumb stick of the driver's joystick.
      // The drive controller is negated here due to the y-axes of the joystick being
      // opposite by default.
      double axisDriveLeftY =
          -m_controllerDrive.getRawAxis(DriveStation.kIDAxisLeftY);
      // Right thumb stick of the driver's joystick.
      double axisDriveRightX =
          m_controllerDrive.getRawAxis(DriveStation.kIDAxisRightX);
      // Left trigger of the driver's joystick.
      double axisDriveLT = m_controllerDrive.getRawAxis(DriveStation.kIDAxisLT);
      // Right trigger of the driver's joystick.
      double axisDriveRT = m_controllerDrive.getRawAxis(DriveStation.kIDAxisRT);

      // Setting robot drive speed
      if (axisDriveLT > 0.5) {
        m_differentialDrive.arcadeDrive(
            axisDriveLeftY * Constants.kMultiplierSlowSpeed,
            axisDriveRightX * Constants.kMultiplierSlowSpeed);
      } else if (axisDriveRT > 0.5) {
        m_differentialDrive.arcadeDrive(
            axisDriveLeftY * Constants.kMultiplierHighSpeed,
            axisDriveRightX * Constants.kMultiplierHighSpeed);
      } else {
        m_differentialDrive.arcadeDrive(
            axisDriveLeftY * Constants.kMultiplierNormalSpeed,
            axisDriveRightX * Constants.kMultiplierNormalSpeed);
      }
    }
  }

  /**
   * Determines whether or not the ball can be launched into the power port, and adjusts the robot to
   * make the shot if it can't.
   */
  private void launchBall() {
    double tolerance = ShuffleboardClass.m_entryDistanceTolerence.getDouble(1);
    double horDistanceToHex = m_ultrasonicSensor.get();
    ShuffleboardClass.m_entryDistanceSensor.setDouble(horDistanceToHex);
    double horDistanceToHoop =
        horDistanceToHex + Constants.kHorDistanceHexagonToHoop;

    double error = Constants.kProjectedHorDistanceToApex - horDistanceToHoop;
    if (Math.abs(error) > tolerance) {
      // m_differentialDrive.arcadeDrive(error * Constants.kP, 0);
    } else {
      // TODO.
    }
  }

  /**
   * Configures the conditions for spinning the control panel, and spins it if necessary.
   */
  private void spinControlPanel() {
    if (m_isInControlPanelMode) {
      if (m_controllerManip.getRawButton(DriveStation.kIDButtonRB)) {
        int controlPanelSpinAmountInitial = m_controlPanelSpinAmount;
        // During a match, the amount of revolutions needed to be completed will be
        // specified as either 3, 4, or 5. The selections below display 6, 8, and 10,
        // respectively, because each color is represented twice on the control panel.
        if (m_buttonManipPressA)
          m_controlPanelSpinAmount = 6;
        else if (m_buttonManipPressB)
          m_controlPanelSpinAmount = 8;
        else if (m_buttonManipPressX)
          m_controlPanelSpinAmount = 10;

        if (controlPanelSpinAmountInitial != m_controlPanelSpinAmount)
          ShuffleboardClass.m_entryTargetSpin
              .setDouble(m_controlPanelSpinAmount);
      } else {
        String targetControlPanelColorInitial = m_targetControlPanelColor;
        if (m_buttonManipPressA)
          m_targetControlPanelColor = "Green";
        else if (m_buttonManipPressB)
          m_targetControlPanelColor = "Red";
        else if (m_buttonManipPressX)
          m_targetControlPanelColor = "Blue";
        else if (m_buttonManipPressY)
          m_targetControlPanelColor = "Yellow";
        if (targetControlPanelColorInitial != m_targetControlPanelColor)
          ShuffleboardClass.m_entryTargetColor
              .setString(m_targetControlPanelColor);
      }

      Color detectedColor = m_colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      if (match.color == kBlueTarget)
        m_detectedColorString = "Blue";
      else if (match.color == kRedTarget)
        m_detectedColorString = "Red";
      else if (match.color == kGreenTarget)
        m_detectedColorString = "Green";
      else if (match.color == kYellowTarget)
        m_detectedColorString = "Yellow";
      else
        m_detectedColorString = "Unknown";
      ShuffleboardClass.m_entryDetectedColor.setString(m_detectedColorString);
      ShuffleboardClass.m_entryConfidence.setDouble(match.confidence);
      turnControlPanel();
      ShuffleboardClass.m_entryTargetSpin.setDouble(m_controlPanelSpinAmount);
      m_lastDetectedColorString = m_detectedColorString;
    } else {
      ShuffleboardClass.m_entryDetectedColor.setString("N/A");
      ShuffleboardClass.m_entryConfidence.setDouble(0);
    }
  }

  /**
   * Turns the control panel when called upon in spinControlPanel().
   */
  public void turnControlPanel() {
    if (m_targetControlPanelColor != m_detectedColorString
        || m_controlPanelSpinAmount > 0)
      m_motorControlPanel.set(Constants.kSpeedControlPanel);
    else
      m_motorControlPanel.set(0);

    if (m_targetControlPanelColor == m_detectedColorString
        && m_lastDetectedColorString != m_detectedColorString
        && m_controlPanelSpinAmount > 0)
      m_controlPanelSpinAmount -= 1;
  }

  /**
   * Initializes test mode.
   */
  @Override
  public void testInit() {
  }

  /**
   * Maintains test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
