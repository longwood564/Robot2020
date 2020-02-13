package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Talons and Victors
  // @TODO: Document these ports.
  private final WPI_TalonSRX rightTalon = new WPI_TalonSRX(6);
  private final WPI_TalonSRX leftTalon = new WPI_TalonSRX(5);
  private final WPI_VictorSPX rightVictor = new WPI_VictorSPX(1);
  private final WPI_VictorSPX leftVictor = new WPI_VictorSPX(4);

  // Differential Drive
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftTalon, rightTalon);

  // Joysticks
  private final Joystick driveController = new Joystick(0);
  private final Joystick manipulateController = new Joystick(1);

  // Drive Motors
  private static final double slowSpeed = 0.5;
  private static final double highSpeed = 0.75;

  // Color Sensor and Wheel (Control Panel)
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private boolean isLookingForColorGreen = false;
  private boolean isLookingForColorRed = false;
  private boolean isLookingForColorBlue = false;
  private boolean isLookingForColorYellow = false;
  private boolean isInControlPanelMode = false;
  private int controlPanelSpinAmount;

 // Encoder (DIO pins 0 and 1)
  Encoder encoder = new Encoder(0,1);

 // Ultrasonic Sensor (distance in inches the robot wants to stay from an object)
 private static final double kHoldDistance = 12.0; 
 // Ultrasonic Sensor (factor to convert sensor values to a distance in inches)
  private static final double kValueToInches = 0.125;
  // Ultrasonic Sensor (proportional speed constant)
  private static final double kP = 0.05;
   
  private static final int kLeftMotorPort = 0;
  private static final int kRightMotorPort = 1;
  private static final int kUltrasonicPort = 0;
   
  private final AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Slave follows master
    rightVictor.follow(rightTalon);
    leftVictor.follow(leftTalon);

    // Color Sensor
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    // Configure Encoder Parameters (current parameters are derived from sample code. will be updated for our needs ASAP.)
   encoder.setDistancePerPulse(4./256.);
   encoder.setMaxPeriod(.1);
   encoder.setMinRate(10);
   encoder.setReverseDirection(true);
   encoder.setSamplesToAverage(5);
 
   encoder.reset();
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
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
    driveSpeed();
    selectColor();
    selectControlPanelSpinAmount();
    colorDetector();

    // sensor returns a value from 0-4095 that is scaled to inches
    double currentDistance = m_ultrasonic.getValue() * kValueToInches;
 
    // convert distance error to a motor speed
    double currentSpeed = (kHoldDistance - currentDistance) * kP;
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function drives the robot at a certain speed.
   */
  private void driveSpeed() {
    if (!isInControlPanelMode) {
      // Left thumb stick
      double rawAxis1 = driveController.getRawAxis(1);
      // Right thumb stick
      double rawAxis4 = driveController.getRawAxis(4);
      // @TODO: Document this axis.
      double rawAxis2 = driveController.getRawAxis(2);
      // @TODO: Document this axis.
      double rawAxis3 = driveController.getRawAxis(3);

      // Setting robot drive speed
      if (rawAxis2 > 0.5) {
        differentialDrive.arcadeDrive(rawAxis1 * slowSpeed, rawAxis4 * slowSpeed);
      } else if (rawAxis3 > 0.5) {
        differentialDrive.arcadeDrive(rawAxis1 * highSpeed, rawAxis4 * highSpeed);
      } else {
        differentialDrive.arcadeDrive(rawAxis1, rawAxis4);
      }
    }
  }

  /*
   * This function is used to select the color to be targeted on the control panel
   * (color wheel).
   */
  private void selectColor() {
    // Button five (lb) is used as the button to activate color selection.
    boolean lb = manipulateController.getRawButton(5);
    // Button one (A) selects the color green.
    boolean a = manipulateController.getRawButton(1);
    // Button two (B) selects the color red.
    boolean b = manipulateController.getRawButton(2);
    // Button three (X) selects the color blue.
    boolean x = manipulateController.getRawButton(3);
    // Button four (Y) selects the color yellow.
    boolean y = manipulateController.getRawButton(4);
    // Button six (rb) is used to select spin amount, which cannot be called here.
    boolean rb = manipulateController.getRawButton(6);

    if (lb) {
      isInControlPanelMode = !isInControlPanelMode;
    }

    if (isInControlPanelMode && !rb) {
      if (a) {
        isLookingForColorGreen = true;
      } else if (b || x || y) {
        isLookingForColorGreen = false;
      }
      if (b) {
        isLookingForColorRed = true;
      } else if (a || x || y) {
        isLookingForColorRed = false;
      }
      if (x) {
        isLookingForColorBlue = true;
      } else if (a || b || y) {
        isLookingForColorBlue = false;
      }
      if (y) {
        isLookingForColorYellow = true;
      } else if (a || b || x) {
        isLookingForColorYellow = false;
      }
    }
  }

  /**
   * This function selects how many spins are required on the control panel (color
   * wheel).
   */
  private void selectControlPanelSpinAmount() {
    // Button six (rb) is used as the button to activate spin amount selector.
    boolean rb = manipulateController.getRawButton(6);
    // Button one (A) selects the spin amount to three.
    boolean a = manipulateController.getRawButton(1);
    // Button one (A) selects the spin amount to four.
    boolean b = manipulateController.getRawButton(2);
    // Button one (A) selects the spin amount to five.
    boolean x = manipulateController.getRawButton(3);

    if (isInControlPanelMode && rb) {
      if (a) {
        controlPanelSpinAmount = 3;
      }
      if (b) {
        controlPanelSpinAmount = 4;
      }
      if (x) {
        controlPanelSpinAmount = 5;
      }
    }
  }

  /**
   * This function detects color using the REVRobotics library and sensor, and
   * then conditionally spins the control panel (color wheel).
   */
  private void colorDetector() {
    if (isInControlPanelMode) {
      String colorString;
      Color detectedColor = m_colorSensor.getColor();
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
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
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);

      if (isLookingForColorGreen) {
        if (colorString != "Green" || controlPanelSpinAmount > 0) {
          // @TODO: Add wheel spinner code (currently waiting for more details on this).
        }
      }
      if (isLookingForColorRed) {
        if (colorString != "Red" || controlPanelSpinAmount > 0) {
          // @TODO: Add wheel spinner code (currently waiting for more details on this).
        }
      }
      if (isLookingForColorBlue) {
        if (colorString != "Blue" || controlPanelSpinAmount > 0) {
          // @TODO: Add wheel spinner code (currently waiting for more details on this).
        }
      }
      if (isLookingForColorYellow) {
        if (colorString != "Yellow" || controlPanelSpinAmount > 0) {
          // @TODO: Add wheel spinner code (currently waiting for more details on this).
        }
      }
    }
  }
}
