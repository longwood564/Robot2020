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
  WPI_TalonSRX rightTalon = new WPI_TalonSRX(6);
  WPI_TalonSRX leftTalon = new WPI_TalonSRX(5);
  WPI_VictorSPX rightVictor = new WPI_VictorSPX(1);
  WPI_VictorSPX leftVictor = new WPI_VictorSPX(4);

  // Differential Drive
  DifferentialDrive differentialDrive = new DifferentialDrive(leftTalon, rightTalon);

  // Joysticks
  Joystick driveController = new Joystick(0);
  Joystick manipulateController = new Joystick(1);

  // Drive Motors
  private static final double slowSpeed = 0.5;
  private static final double highSpeed = 0.75;

  // Color Sensor and Wheel (Control Panel)
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  boolean isLookingForColorGreen, isLookingForColorRed, isLookingForColorBlue, isLookingForColorYellow = false;
  boolean isInControlPanelMode = false;
  int controlPanelSpins;

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
  public void driveSpeed() {
    if (!isInControlPanelMode) {
      // Left thumb stick
      double rawAxis1 = driveController.getRawAxis(1);
      // Right thumb stick
      double rawAxis4 = driveController.getRawAxis(4);
      double rawAxis2 = driveController.getRawAxis(2);
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
   * This function is used to select the color for the color wheel.
   */
  public void selectColor() {
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

    if (isInControlPanelMode) {
      if (a && !rb) {
        isLookingForColorGreen = true;
      } else if (b || x || y) {
        isLookingForColorGreen = false;
      }
      if (b && !rb) {
        isLookingForColorRed = true;
      } else if (a || x || y) {
        isLookingForColorRed = false;
      }
      if (x && !rb) {
        isLookingForColorBlue = true;
      } else if (a || b || y) {
        isLookingForColorBlue = false;
      }
      if (y && !rb) {
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
  public void selectControlPanelSpinAmount() {
    // Button six (rb) is used as the button to activate spin amount selector.
    boolean rb = manipulateController.getRawButton(6);
    // Button one (A) selects the spin amount to three.
    boolean a = manipulateController.getRawButton(1);
    // Button one (A) selects the spin amount to four.
    boolean b = manipulateController.getRawButton(2);
    // Button one (A) selects the spin amount to five.
    boolean x = manipulateController.getRawButton(3);

    if (isInControlPanelMode) {
      if (rb && a) {
        controlPanelSpins = 3;
      }
      if (rb && b) {
        controlPanelSpins = 4;
      }
      if (rb && x) {
        controlPanelSpins = 5;
      }
    }
  }

  /**
   * This function detects color using the REVRobotics library and sensor.
   */
  public void colorDetector() {
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
        while (colorString != "Green" || controlPanelSpins > 0) {
          // Wheel spinner = true
        }
      }
      if (isLookingForColorRed || controlPanelSpins > 0) {
        while (colorString != "Red") {
          // Wheel spinner = true
        }
      }
      if (isLookingForColorBlue || controlPanelSpins > 0) {
        while (colorString != "Blue") {
          // Wheel spinner = true
        }
      }
      if (isLookingForColorYellow || controlPanelSpins > 0) {
        while (colorString != "Yellow") {
          // Wheel spinner = true
        }
      }
    }
  }
}
