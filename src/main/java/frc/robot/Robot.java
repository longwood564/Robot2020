package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Joysticks
  private final Joystick driveController = new Joystick(0);
  private final Joystick manipController = new Joystick(1);
  private boolean manipAPress = false;
  private boolean manipBPress = false;
  private boolean manipXPress = false;
  private boolean manipYPress = false;
  private boolean manipLBPress = false;

  // State
  private boolean isInControlPanelMode = false;

  // Autonomous
  private static final String kDefaultAuto = "Default";
  private String selectedAuto;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  // Driving
  private final WPI_TalonSRX rightTalon = new WPI_TalonSRX(6);
  private final WPI_TalonSRX leftTalon = new WPI_TalonSRX(5);
  private final WPI_VictorSPX rightVictor = new WPI_VictorSPX(1);
  private final WPI_VictorSPX leftVictor = new WPI_VictorSPX(4);
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftTalon, rightTalon);
  private static final double slowSpeed = 0.5;
  private static final double highSpeed = 0.75;
  private static final double defaultSpeed = 0.65;

  // Launching
  // TODO: Setup both of these.
  WPI_TalonSRX leftLauncher = new WPI_TalonSRX(7);
  WPI_TalonSRX rightLauncher = new WPI_TalonSRX(8);

  // Color Sensing
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  private static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private String targetControlPanelColor = "N/A";
  private int controlPanelSpinAmount = 0;

  // Vision

  // Shuffleboard General
  private final ShuffleboardTab generalTab = Shuffleboard.getTab("General");
  private final ShuffleboardLayout stateLayout = generalTab.getLayout("State", BuiltInLayouts.kGrid).withPosition(0, 0)
      .withSize(3, 1).withProperties(Map.of("Number of columns", 1, "Number of rows", 1));
  private final NetworkTableEntry controlPanelModeEntry = stateLayout.add("Control panel mode", false)
      .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  private final ShuffleboardLayout autonomousLayout = generalTab.getLayout("Autonomous", BuiltInLayouts.kGrid)
      .withPosition(3, 0).withSize(3, 1)
      .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 1, "Number of rows", 1));
  private final ShuffleboardLayout drivingLayout = generalTab.getLayout("Driving", BuiltInLayouts.kGrid)
      .withPosition(0, 1).withSize(3, 3).withProperties(Map.of("Number of columns", 1, "Number of rows", 1));
  private final ShuffleboardLayout launchingLayout = generalTab.getLayout("Launching", BuiltInLayouts.kGrid)
      .withPosition(3, 1).withSize(1, 1).withProperties(Map.of("Number of columns", 1, "Number of rows", 1));
  private final ShuffleboardLayout controlPanelLayout = generalTab.getLayout("Color Sensing", BuiltInLayouts.kGrid)
      .withPosition(3, 2).withSize(3, 2).withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
  private final NetworkTableEntry detectedColorEntry = controlPanelLayout.add("Detected color", "N/A").getEntry();
  private final NetworkTableEntry confidenceEntry = controlPanelLayout.add("Confidence", 0).getEntry();
  private final NetworkTableEntry targetColorEntry = controlPanelLayout.add("Target Color", "N/A").getEntry();
  private final NetworkTableEntry targetSpinEntry = controlPanelLayout.add("Target Spins", 0).getEntry();
  private final ShuffleboardLayout visionLayout = generalTab.getLayout("Vision", BuiltInLayouts.kGrid)
      .withPosition(6, 0).withSize(1, 1).withProperties(Map.of("Number of columns", 1, "Number of rows", 1));

  // Shuffleboard Tools
  private final ShuffleboardTab toolsTab = Shuffleboard.getTab("Tools");
  private final ShuffleboardLayout launchingToolsLayout = toolsTab.getLayout("Launching Tools", BuiltInLayouts.kGrid)
      .withPosition(0, 0).withSize(4, 5).withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
  private final ShuffleboardLayout projectileMotionPredLayout = launchingToolsLayout
      .getLayout("Projectile Motion Prediction", BuiltInLayouts.kList).withSize(2, 5);
  private final NetworkTableEntry horizontalDistanceEntry = projectileMotionPredLayout
      .addPersistent("Horizontal Distance (m)", 0).getEntry();
  private final NetworkTableEntry runPredEntry = projectileMotionPredLayout.add("Calculate", false)
      .withWidget(BuiltInWidgets.kToggleButton).getEntry();
  private final NetworkTableEntry verticalDistanceEntry = projectileMotionPredLayout.add("Vertical Distance (m)", 0)
      .withWidget(BuiltInWidgets.kTextView).getEntry();
  private final ShuffleboardLayout projectileMotionSimLayout = launchingToolsLayout
      .getLayout("Projectile Motion Simulation", BuiltInLayouts.kList).withSize(2, 5);
  private double initialVelocitySim = 0;
  private final NetworkTableEntry initialVelocityEntry = projectileMotionSimLayout
      .addPersistent("Initial Velocity (ms^-1)", initialVelocitySim).getEntry();
  private double initialAngleSim = 0;
  private double initialAngleSimRad = 0;
  private final NetworkTableEntry initialAngleEntry = projectileMotionSimLayout
      .addPersistent("Initial Angle (Degrees)", initialAngleSim).getEntry();
  private final NetworkTableEntry runSimEntry = projectileMotionSimLayout.add("Run Simulation", false)
      .withWidget(BuiltInWidgets.kToggleButton).getEntry();
  private final NetworkTableEntry simGraphEntry = projectileMotionSimLayout.add("Simlulation", new double[] { 0, 0 })
      .withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible time", 7)).getEntry();
  private final NetworkTableEntry simTimeEntry = projectileMotionSimLayout.add("Simulation Time (s)", 0)
      .withWidget(BuiltInWidgets.kTextView).getEntry();
  private final Timer simTimer = new Timer();
  private boolean runningSim = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Slave follows master
    rightVictor.follow(rightTalon);
    leftVictor.follow(leftTalon);

    // Add color sensor matches.
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    // Add Shuffleboard sendables. We define the NetworkTableEntry objects as member
    // variables when adding those widgets because we need to access them to update
    // them. Contrary, we don't assign these widgets to any variables because, as
    // Sendable interfaces, they will automatically be updated. There is a
    // distinction to be made between assigning the ComplexWidget to a variable, and
    // assigning the SendableChooser to a variable - which we *do* do.
    autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    autonomousLayout.add(autoChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
    drivingLayout.add(differentialDrive);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (runPredEntry.getBoolean(false)) {
      runPredEntry.setBoolean(false);
      double horDistance = horizontalDistanceEntry.getDouble(0);
      // This expression calculates how high the ball will be at a specified distance
      // away from the robot. See the research document for the derivation of the
      // formula used here.
      verticalDistanceEntry
          .setDouble(horDistance * Math.tan(Constants.kLauncherAngle) - (0.5 * Constants.kAccelDueToGravity
              * Math.pow((horDistance / (Constants.kInitialVelocityBall * Math.cos(Constants.kLauncherAngle))), 2)));
    }

    if (runSimEntry.getBoolean(false) && !runningSim) {
      runningSim = true;
      // Only set these once. These can't change in the middle of a simulation.
      initialVelocitySim = initialVelocityEntry.getDouble(initialVelocitySim);
      initialAngleSimRad = Math.toRadians(initialAngleEntry.getDouble(initialAngleSim));
      simTimer.start();
    } else if (runSimEntry.getBoolean(false) && runningSim) {
      double time = simTimer.get();
      double horizontalDistance = (initialVelocitySim * Math.cos(initialAngleSimRad)) * time;
      double verticalDistance = (initialVelocitySim * Math.sin(initialAngleSimRad)) * time
          + 0.5 * -Constants.kAccelDueToGravity * Math.pow(time, 2);
      if (verticalDistance < 0) {
        runningSim = false;
        runSimEntry.setBoolean(false);
        simTimer.reset();
      } else {
        simGraphEntry.setDoubleArray(new double[] { horizontalDistance, verticalDistance });
        simTimeEntry.setDouble(time);
      }
    } else if (!runSimEntry.getBoolean(false) && runningSim) {
      // Cancel a running simulation.
      runningSim = false;
      simTimer.reset();
      simGraphEntry.setDoubleArray(new double[] { 0, 0 });
    }
  }

  /**
   * This function is called when initializing disabled mode.
   */
  @Override
  public void disabledInit() {
    detectedColorEntry.setString("N/A");
    confidenceEntry.setDouble(0);
    targetColorEntry.setString("N/A");
    targetSpinEntry.setDouble(0);
  }

  /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    selectedAuto = autoChooser.getSelected();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (selectedAuto) {
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
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
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function reads pressed states from the gamepads. This is done here
   * because it is paramount that getRawButtonPressed() is only called once per
   * loop, because, the second time, it will more than likely just return "false"
   * for any button.
   */
  private void updateInputs() {
    manipAPress = manipController.getRawButtonPressed(1);
    manipBPress = manipController.getRawButtonPressed(2);
    manipXPress = manipController.getRawButtonPressed(3);
    manipYPress = manipController.getRawButtonPressed(4);
    manipLBPress = manipController.getRawButtonPressed(5);
  }

  /**
   * This function handles general state of the teleoperated mode.
   */
  private void handleState() {
    if (manipLBPress) {
      isInControlPanelMode = !isInControlPanelMode;
      controlPanelModeEntry.setBoolean(isInControlPanelMode);
    } else {
      isInControlPanelMode = controlPanelModeEntry.getBoolean(isInControlPanelMode);
    }
  }

  /**
   * This function drives the robot at a certain speed.
   */
  private void driveSpeed() {
    if (isInControlPanelMode) {
      // Explicitly stop the motors since we are in control panel mode, and do not
      // need to be moving. This is necessary because the motor power must be updated
      // for every iteration of the loop.
      differentialDrive.stopMotor();
    } else {
      // Left thumb stick of the manipulator's joystick.
      // The drive controller is negated here due to the y-axes of the joystick being
      // opposite by default.
      double rawAxis1 = -driveController.getRawAxis(1);
      // Right thumb stick of the manipulator's joystick.
      double rawAxis4 = driveController.getRawAxis(4);
      // Left trigger of the manipulator's joystick.
      double rawAxis2 = driveController.getRawAxis(2);
      // Right trigger of the manipulator's joystick.
      double rawAxis3 = driveController.getRawAxis(3);

      // Setting robot drive speed
      if (rawAxis2 > 0.5) {
        differentialDrive.arcadeDrive(rawAxis1 * slowSpeed, rawAxis4 * slowSpeed);
      } else if (rawAxis3 > 0.5) {
        differentialDrive.arcadeDrive(rawAxis1 * highSpeed, rawAxis4 * highSpeed);
      } else {
        differentialDrive.arcadeDrive(rawAxis1 * defaultSpeed, rawAxis4 * defaultSpeed);
      }
    }
  }

  /**
   * This function determines whether or not the ball can be launched into the
   * power port, and adjusts the robot to make the shot if it can't.
   */
  private void launchBall() {
    // TODO: Allow for this to be configurable from the DS.
    double tolerance = 1;
    // TODO: Plug in the ultrasonic sensor reading here.
    double horDistanceToHex = 13;
    double horDistanceToHoop = horDistanceToHex + Constants.kHorDistanceHexagonToHoop;

    // double projectedVertDistanceToHex = calculateHeight(horDistanceToHex);
    // double projectedHeightToHoop = calculateHeight(horDistanceToHoop);

    // if (Math.abs(Constants.kVertDistanceLauncherToHex -
    // projectedVertDistanceToHex) > tolerance
    // || Math.abs(Constants.kVertDistanceLauncherToHoop - projectedHeightToHoop) >
    // tolerance) {
    // if (Constants.kProjectedHorDistanceToApex > horDistanceToHoop) {
    // // Too close.
    // } else {
    // // Too far.
    // }
    // } else {
    // // Shot is lined up!
    // }
  }

  /**
   * This function configures the conditions for spinning the control panel, and
   * spins it if necessary.
   */
  private void spinControlPanel() {
    if (isInControlPanelMode) {
      boolean manipRB = manipController.getRawButton(6);
      if (manipRB) {
        int controlPanelSpinAmountInitial = controlPanelSpinAmount;
        if (manipAPress) {
          controlPanelSpinAmount = 3;
        } else if (manipBPress) {
          controlPanelSpinAmount = 4;
        } else if (manipXPress) {
          controlPanelSpinAmount = 5;
        }
        if (controlPanelSpinAmountInitial != controlPanelSpinAmount) {
          targetSpinEntry.setDouble(controlPanelSpinAmount);
        }
      } else {
        String targetControlPanelColorInitial = targetControlPanelColor;
        if (manipAPress) {
          targetControlPanelColor = "Green";
        } else if (manipBPress) {
          targetControlPanelColor = "Red";
        } else if (manipXPress) {
          targetControlPanelColor = "Blue";
        } else if (manipYPress) {
          targetControlPanelColor = "Yellow";
        }
        if (targetControlPanelColorInitial != targetControlPanelColor) {
          targetColorEntry.setString(targetControlPanelColor);
        }
      }

      String colorString;
      Color detectedColor = colorSensor.getColor();
      ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
      if (match.color == kBlueTarget) {
        colorString = "Blue";
      } else if (match.color == kRedTarget) {
        colorString = "Red";
      } else if (match.color == kGreenTarget) {
        colorString = "Green";
      } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } else {
        colorString = "Unknown";
      }
      detectedColorEntry.setString(colorString);
      confidenceEntry.setDouble(match.confidence);

      double doubleContolPanelSpinAmount = controlPanelSpinAmount * 2;
      if (targetControlPanelColor == "Green" && (colorString != "Green" || doubleContolPanelSpinAmount > 0)) {
        // @TODO: Add wheel spinner code (currently waiting for more details on this).
      } else if (targetControlPanelColor == "Red" && (colorString != "Red" || doubleContolPanelSpinAmount > 0)) {
        // @TODO: Add wheel spinner code (currently waiting for more details on this).
      } else if (targetControlPanelColor == "Blue" && (colorString != "Blue" || doubleContolPanelSpinAmount > 0)) {
        // @TODO: Add wheel spinner code (currently waiting for more details on this).
      } else if (targetControlPanelColor == "Yellow" && (colorString != "Yellow" || doubleContolPanelSpinAmount > 0)) {
        // @TODO: Add wheel spinner code (currently waiting for more details on this).
      }
      targetSpinEntry.setDouble(controlPanelSpinAmount);
    } else {
      detectedColorEntry.setString("N/A");
      confidenceEntry.setDouble(0);
    }
  }
}
