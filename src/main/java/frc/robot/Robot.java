package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
  private boolean m_buttonDrivePressA = false;
  private boolean m_buttonDrivePressB = false;
  private boolean m_buttonManipPressA = false;
  private boolean m_buttonManipPressB = false;
  private boolean m_buttonManipPressX = false;
  private boolean m_buttonManipPressY = false;
  private boolean m_buttonManipPressBack = false;
  private boolean m_buttonManipPressStart = false;
  private int m_povLastLoop = -1;
  private boolean m_buttonManipPressDpadLeft = false;
  private boolean m_buttonManipPressDpadUp = false;
  private boolean m_buttonManipPressDpadRight = false;
  private boolean m_buttonManipPressDpadDown = false;

  // State
  private boolean m_isInLaunchingMode = false;
  private boolean m_isInLaunchingModeLastLoop = false;
  private boolean m_isInControlPanelMode = false;
  private boolean m_isInControlPanelModeLastLoop = false;

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

  // Ball Intake
  private final WPI_TalonSRX m_motorIntake =
      new WPI_TalonSRX(RoboRIO.kPortMotorIntake);
  private final WPI_VictorSPX m_motorBelt =
      new WPI_VictorSPX(RoboRIO.kPortMotorBelt);
  private final DigitalInput m_photoelectricSensorEnter =
      new DigitalInput(RoboRIO.kPortPhotoelectricSensorEnter);
  private final DigitalInput m_photoelectricSensorExit =
      new DigitalInput(RoboRIO.kPortPhotoelectricSensorExit);
  private int m_ballsInStorage = 0;
  private boolean m_ballDetectedEnterLastLoop = false;
  private boolean m_ballDetectedExitLastLoop = false;
  
  // Winch
  private final WPI_VictorSPX m_motorWinch =
      new WPI_VictorSPX(RoboRIO.kPortMotorWinch);
  private final DoubleSolenoid m_doubleSolenoidWinch =
      new DoubleSolenoid(RoboRIO.kPortDoubleSolenoidForwardWinch,
          RoboRIO.kPortDoubleSolenoidBackwardWinch);
  private boolean m_raiseWinch = false;

  // Launching
  WPI_VictorSPX m_motorLauncherLeft =
      new WPI_VictorSPX(RoboRIO.kPortMotorLauncherLeft);
  WPI_TalonSRX m_motorLauncherRight =
      new WPI_TalonSRX(RoboRIO.kPortMotorLauncherRight);
  private final AnalogInput m_analogInputUltrasonicSensor =
      new AnalogInput(RoboRIO.kPortUltrasonicSensorPort);
  // Leave this uninitialized because we have to configure the analog input.
  private AnalogPotentiometer m_ultrasonicSensor;
  private boolean m_launchBall = false;

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
  private final DoubleSolenoid m_doubleSolenoidControlPanel =
      new DoubleSolenoid(RoboRIO.kPortDoubleSolenoidForwardControlPanel,
          RoboRIO.kPortDoubleSolenoidBackwardControlPanel);
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

    ShuffleboardHelper.shuffleboardInit();
    // Add Shuffleboard sendables. We define the NetworkTableEntry objects as member
    // variables when adding those widgets because we need to access them to update
    // them. Contrary, we don't assign these widgets to any variables because, as
    // Sendable interfaces, they will automatically be updated. There is a
    // distinction to be made between assigning the ComplexWidget to a variable, and
    // assigning the SendableChooser to a variable - which we *do* do.
    m_autoChooser.setDefaultOption("Default Auto", kAutoCaseDefault);
    ShuffleboardHelper.m_layoutAutonomous.add(m_autoChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);
    ShuffleboardHelper.m_layoutDriving.add(m_differentialDrive);
    ShuffleboardHelper.m_layoutLaunching
        .add("Optimal Distance to Apex",
            Constants.kProjectedHorDistanceToApex
                - Constants.kHorDistanceHexagonToHoop)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(ShuffleboardHelper.kPropertiesDistanceSensor)
        .getEntry();
  }

  /**
   * Updates diagnostics while the robot power is on.
   */
  @Override
  public void robotPeriodic() {
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
    ShuffleboardHelper.updateSimulations();
  }

  /**
   * Initializes disabled mode. This method is responsible for resetting state to the way it this
   * class is when it's initialized.
   */
  @Override
  public void disabledInit() {
    m_isInControlPanelMode = false;
    // Force a state change.
    m_isInControlPanelModeLastLoop = true;
    ShuffleboardHelper.m_entryControlPanelMode
        .setBoolean(m_isInControlPanelMode);
    m_isInLaunchingMode = false;
    // Force a state change.
    m_isInLaunchingModeLastLoop = true;
    ShuffleboardHelper.m_entryLaunchingMode.setBoolean(m_isInLaunchingMode);
    // Running this method will update Shuffleboard to show "N/A" and such, which is desirable while the
    // robot is disabled.
    handleState();

    m_ballsInStorage = 0;
    m_ballDetectedEnterLastLoop = false;
    m_ballDetectedExitLastLoop = false;
    ShuffleboardHelper.m_entryBallsInStorage.setDouble(m_ballsInStorage);
    ShuffleboardHelper.m_entryBallDetectedEnter.setBoolean(false);
    ShuffleboardHelper.m_entryBallDetectedExit.setBoolean(false);
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
    m_doubleSolenoidControlPanel.set(DoubleSolenoid.Value.kReverse);
    m_doubleSolenoidWinch.set(DoubleSolenoid.Value.kReverse);

    m_isInControlPanelMode = false;
    m_isInControlPanelModeLastLoop = false;
    m_raiseWinch = false;

    m_detectedColorString = "N/A";
    m_lastDetectedColorString = "N/A";
    m_targetControlPanelColor = "N/A";
    m_controlPanelSpinAmount = 0;

    m_compressor.start();

    disabledInit();
  }

  /**
   * Maintains teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    updateInputs();
    handleState();
    driveSpeed();
    intakeBalls();
    launchBalls();
    toggleWinch();
    spinControlPanel();
  }

  /**
   * Reads pressed states from the gamepads. This is done here because it is paramount that
   * getRawButtonPressed() is only called once per loop, because, the second time, it will more than
   * likely just return "false" for any button.
   */
  private void updateInputs() {
    // Driver controller updates.
    m_buttonDrivePressA =
        m_controllerDrive.getRawButtonPressed(DriveStation.kIDButtonA);
    m_buttonDrivePressB =
        m_controllerDrive.getRawButtonPressed(DriveStation.kIDButtonB);

    // Manipulator controller updates.
    m_buttonManipPressA =
        m_controllerManip.getRawButtonPressed(DriveStation.kIdButtonA);
    m_buttonManipPressB =
        m_controllerManip.getRawButtonPressed(DriveStation.kIdButtonB);
    m_buttonManipPressX =
        m_controllerManip.getRawButtonPressed(DriveStation.kIdButtonX);
    m_buttonManipPressY =
        m_controllerManip.getRawButtonPressed(DriveStation.kIdButtonY);
    m_buttonManipPressBack =
        m_controllerManip.getRawButtonPressed(DriveStation.kIdButtonBack);
    m_buttonManipPressStart =
        m_controllerManip.getRawButtonPressed(DriveStation.kIdButtonStart);
    int pov = m_controllerManip.getPOV(DriveStation.kIdPovDpad);
    if (pov != -1 && pov == m_povLastLoop)
      pov = -1;
    m_buttonManipPressDpadLeft = pov == 270;
    m_buttonManipPressDpadUp = pov == 0;
    m_buttonManipPressDpadRight = pov == 90;
    m_buttonManipPressDpadDown = pov == 180;
    m_povLastLoop = pov;
  }

  /**
   * Handles general state of the teleoperated mode.
   */
  private void handleState() {
    if (m_buttonManipPressBack) {
      m_isInLaunchingMode = !m_isInLaunchingMode;
      ShuffleboardHelper.m_entryLaunchingMode.setBoolean(m_isInLaunchingMode);
    } else {
      m_isInLaunchingMode = ShuffleboardHelper.m_entryLaunchingMode
          .getBoolean(m_isInLaunchingMode);
    }
    if (m_buttonManipPressStart) {
      m_isInControlPanelMode = !m_isInControlPanelMode;
      ShuffleboardHelper.m_entryControlPanelMode
          .setBoolean(m_isInControlPanelMode);
    } else {
      m_isInControlPanelMode = ShuffleboardHelper.m_entryControlPanelMode
          .getBoolean(m_isInControlPanelMode);
    }

    // If launching or control panel mode is enabled and the robot is driven via controller, disable it.
    if (Math.abs(m_controllerDrive.getRawAxis(DriveStation.kIdAxisLeftY)) > 0.5
        || Math.abs(
            m_controllerDrive.getRawAxis(DriveStation.kIdAxisRightX)) > 0.5) {
      if (m_isInControlPanelMode) {
        m_isInControlPanelMode = false;
        ShuffleboardHelper.m_entryControlPanelMode
            .setBoolean(m_isInControlPanelMode);
      } else if (m_isInLaunchingMode) {
        m_isInLaunchingMode = false;
        ShuffleboardHelper.m_entryLaunchingMode.setBoolean(m_isInLaunchingMode);
      }
    }

    // Set the control panel values to their defaults when not enabled.
    if (m_isInControlPanelModeLastLoop != m_isInControlPanelMode) {
      if (m_isInControlPanelMode) {
        // Disallow being in both modes simultaneously.
        if (m_isInLaunchingMode) {
          m_isInLaunchingMode = false;
          ShuffleboardHelper.m_entryLaunchingMode.setBoolean(false);
        }
      } else {
        m_detectedColorString = "N/A";
        m_lastDetectedColorString = "N/A";
        m_targetControlPanelColor = "N/A";
        m_controlPanelSpinAmount = 0;
        ShuffleboardHelper.m_entryDetectedColor
            .setString(m_detectedColorString);
        ShuffleboardHelper.m_entryTargetColor
            .setString(m_targetControlPanelColor);
        ShuffleboardHelper.m_entryTargetSpin
            .setDouble(m_controlPanelSpinAmount);
        ShuffleboardHelper.m_entryConfidence.setDouble(0);
      }
    }
    // Set the launching values to their defaults when not enabled.
    if (m_isInLaunchingModeLastLoop != m_isInLaunchingMode) {
      if (m_isInLaunchingMode) {
        // Disallow being in both modes simultaneously.
        if (m_isInControlPanelMode) {
          m_isInControlPanelMode = false;
          ShuffleboardHelper.m_entryControlPanelMode.setBoolean(false);
        }
      } else {
        m_launchBall = false;
        ShuffleboardHelper.m_entryLaunchBall.setBoolean(m_launchBall);
        ShuffleboardHelper.m_entryDistanceSensor.setDouble(0);
      }
    }
    m_isInControlPanelModeLastLoop = m_isInControlPanelMode;
    m_isInLaunchingModeLastLoop = m_isInLaunchingMode;
  }

  /**
   * Drives the robot at a certain speed inputted by the driver. DifferentialDrive squares the input
   * values by default, so in order to apply a speed modifier, we have to square it ourselves, so that
   * it is squared before the modifier is applied.
   */
  private void driveSpeed() {
    // Left thumb stick of the driver's joystick.
    // The drive controller is negated here due to the y-axes of the joystick being
    // opposite by default.
    double axisDriveLeftY =
        -m_controllerDrive.getRawAxis(DriveStation.kIdAxisLeftY);
    double speed = Math.signum(axisDriveLeftY) * Math.pow(axisDriveLeftY, 2);
    // Right thumb stick of the driver's joystick.
    double axisDriveRightX =
        m_controllerDrive.getRawAxis(DriveStation.kIdAxisRightX);
    double zRotation =
        Math.signum(axisDriveRightX) * Math.pow(axisDriveRightX, 2);
    // Left trigger of the driver's joystick.
    double axisDriveLt = m_controllerDrive.getRawAxis(DriveStation.kIdAxisLt);
    // Right trigger of the driver's joystick.
    double axisDriveRt = m_controllerDrive.getRawAxis(DriveStation.kIdAxisRt);

    // Setting robot drive speed.
    if (axisDriveLt > 0.5) {
      m_differentialDrive.arcadeDrive(speed * Constants.kMultiplierSlowSpeed,
          zRotation * Constants.kMultiplierSlowSpeed, false);
    } else if (axisDriveRt > 0.5) {
      m_differentialDrive.arcadeDrive(speed * Constants.kMultiplierHighSpeed,
          zRotation * Constants.kMultiplierHighSpeed, false);
    } else {
      m_differentialDrive.arcadeDrive(speed * Constants.kMultiplierNormalSpeed,
          zRotation * Constants.kMultiplierNormalSpeed, false);
    }
  }

  private void intakeBalls() {
    // If the manipulator holds LT, and the storage isn't full, activate the intake.
    // TODO: Is this ballsInStorage check putting too much trust in the sensor?
    if (m_controllerManip.getRawAxis(DriveStation.kIdAxisLt) > 0.50
        && m_ballsInStorage < 3)
      m_motorIntake.set(Constants.kSpeedIntake);
    else
      m_motorIntake.set(0);

    // Use this state variable to avoid setting the power of the belt motor more than once.
    boolean advanceBelt = false;
    // The digital input returns "true" if the circuit is open. Detecting the
    // object, the power cell, closes the circuit.
    boolean ballDetectedEnter = !m_photoelectricSensorEnter.get();
    boolean ballDetectedExit = !m_photoelectricSensorExit.get();
    // Advance the belt if there's a ball in the enter spot, and more room above.
    if (ballDetectedEnter) {
      if (!m_ballDetectedEnterLastLoop)
        ++m_ballsInStorage;
      if (m_ballsInStorage < 3)
        advanceBelt = true;
      ShuffleboardHelper.m_entryBallDetectedEnter.setBoolean(ballDetectedEnter);
      ShuffleboardHelper.m_entryBallsInStorage.setDouble(m_ballsInStorage);
    } else if (!ballDetectedEnter) {
      ShuffleboardHelper.m_entryBallDetectedEnter.setBoolean(ballDetectedEnter);
      advanceBelt = false;
    }
    // Keep track of balls exiting.
    if (!ballDetectedExit) {
      if (m_ballDetectedExitLastLoop)
        --m_ballsInStorage;
      // Stop launching the balls if we have finished.
      if (m_launchBall && m_ballsInStorage == 0) {
        m_launchBall = false;
        ShuffleboardHelper.m_entryLaunchBall.setBoolean(m_launchBall);
      }
      ShuffleboardHelper.m_entryBallsInStorage.setDouble(m_ballsInStorage);
      ShuffleboardHelper.m_entryBallDetectedExit.setBoolean(ballDetectedExit);
    } else if (ballDetectedExit) {
      ShuffleboardHelper.m_entryBallDetectedExit.setBoolean(ballDetectedExit);
    }

    // If we are ready to launch the ball, override the false advanceBelt from the storage being full.
    // TODO: Check to see if the launcher motor has been revved up.
    if (m_launchBall)
      advanceBelt = true;

    // If a manipulator bumper is held, disregard all of the previous logic, and force a belt movement.
    if (m_controllerManip.getRawButton(DriveStation.kIdButtonRb))
      m_motorBelt.set(Constants.kSpeedBelt);
    else if (m_controllerManip.getRawButton(DriveStation.kIdButtonLb))
      m_motorBelt.set(-Constants.kSpeedBelt);
    // Use the logic based off of the photosensors for belt movement.
    else
      m_motorBelt.set(advanceBelt ? Constants.kSpeedBelt : 0);

    // Rev up the launcher motors as soon as we start collecting balls.
    if (m_ballsInStorage >= 1)
      m_motorLauncherLeft.set(Constants.kSpeedLauncher);
    else
      m_motorLauncherLeft.set(0);

    m_ballDetectedEnterLastLoop = ballDetectedEnter;
    m_ballDetectedExitLastLoop = ballDetectedExit;
  }

  /**
   * Toggles whether or not the winch is raised or declined.
   */
  private void toggleWinch() {
    if (m_buttonDrivePressA) {
      m_raiseWinch = !m_raiseWinch;
    }

    if (m_raiseWinch) {
      m_motorWinch.set(1);
      m_doubleSolenoidWinch.set(DoubleSolenoid.Value.kForward);
    } else {
      m_motorWinch.set(0);
      m_doubleSolenoidWinch.set(DoubleSolenoid.Value.kReverse);
    }
  }

  /**
   * Determines whether or not the ball can be launched into the power port, and adjusts the robot to
   * make the shot if it cannot.
   */
  private void launchBalls() {
    if (m_isInLaunchingMode) {
      double tolerance =
          ShuffleboardHelper.m_entryDistanceTolerence.getDouble(1);
      double horDistanceToHex = m_ultrasonicSensor.get();
      ShuffleboardHelper.m_entryDistanceSensor.setDouble(horDistanceToHex);
      double horDistanceToHoop =
          horDistanceToHex + Constants.kHorDistanceHexagonToHoop;

      double error = Constants.kProjectedHorDistanceToApex - horDistanceToHoop;
      if (Math.abs(error) > tolerance) {
        // TODO: Very experimental! Fine tune this.
        // Cap out the correction speed at the higher driving speed. Don't square the inputs because this
        // isn't from an analog stick, so that kind of precision isn't necessary.
        m_differentialDrive.arcadeDrive(error > 0
            ? Math.min(Constants.kMultiplierHighSpeed, error * Constants.kP)
            : Math.max(-Constants.kMultiplierHighSpeed, error * Constants.kP),
            0, false);
        m_launchBall = false;
        ShuffleboardHelper.m_entryLaunchBall.setBoolean(m_launchBall);
      } else {
        m_differentialDrive.arcadeDrive(0, 0);
        m_launchBall = true;
        ShuffleboardHelper.m_entryLaunchBall.setBoolean(m_launchBall);
      }
    }

    // If the manipulator trigger is held, override our autonomous logic and manually spin up the
    // launcher.
    if (m_controllerDrive.getRawAxis(DriveStation.kIdAxisRt) > 0.5)
      m_motorLauncherLeft.set(Constants.kSpeedLauncher);
  }

  /**
   * Configures the conditions for spinning the control panel, and spins it if necessary.
   */
  private void spinControlPanel() {
    if (m_isInControlPanelMode) {
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
        ShuffleboardHelper.m_entryTargetColor
            .setString(m_targetControlPanelColor);

      int controlPanelSpinAmountInitial = m_controlPanelSpinAmount;
      // During a match, the amount of revolutions needed to be completed will be
      // specified as either 3, 4, or 5. The selections below display 6, 8, and 10,
      // respectively, because each color is represented twice on the control panel.
      if (m_buttonManipPressDpadLeft)
        m_controlPanelSpinAmount = 6;
      else if (m_buttonManipPressDpadUp)
        m_controlPanelSpinAmount = 8;
      else if (m_buttonManipPressDpadRight)
        m_controlPanelSpinAmount = 10;
      else if (m_buttonManipPressDpadDown)
        m_controlPanelSpinAmount = 0;
      if (controlPanelSpinAmountInitial != m_controlPanelSpinAmount)
        ShuffleboardHelper.m_entryTargetSpin
            .setDouble(m_controlPanelSpinAmount);

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
      ShuffleboardHelper.m_entryDetectedColor.setString(m_detectedColorString);
      ShuffleboardHelper.m_entryConfidence.setDouble(match.confidence);
      turnControlPanel();
      ShuffleboardHelper.m_entryTargetSpin.setDouble(m_controlPanelSpinAmount);
      m_lastDetectedColorString = m_detectedColorString;
    }
  }

  /**
   * Turns the control panel when called upon in spinControlPanel().
   */
  private void turnControlPanel() {
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
