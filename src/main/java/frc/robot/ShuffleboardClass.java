package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This class deals with variables relating to Shuffleboard.
 */
public final class ShuffleboardClass {
        // Shuffleboard General
        public static final ShuffleboardTab m_tabGeneral =
                        Shuffleboard.getTab("General");
        public static final ShuffleboardLayout m_layoutState =
                        m_tabGeneral.getLayout("State", BuiltInLayouts.kGrid)
                                        .withPosition(0, 0).withSize(3, 1)
                                        .withProperties(Map.of(
                                                        "Number of columns", 1,
                                                        "Number of rows", 1));
        public static final NetworkTableEntry m_entryControlPanelMode =
                        m_layoutState.add("Control panel mode", false)
                                        .withWidget(BuiltInWidgets.kToggleSwitch)
                                        .getEntry();
        public static final ShuffleboardLayout m_layoutAutonomous = m_tabGeneral
                        .getLayout("Autonomous", BuiltInLayouts.kGrid)
                        .withPosition(3, 0).withSize(3, 1)
                        .withProperties(Map.of("Label position", "HIDDEN",
                                        "Number of columns", 1,
                                        "Number of rows", 1));
        public static final ShuffleboardLayout m_layoutDriving =
                        m_tabGeneral.getLayout("Driving", BuiltInLayouts.kGrid)
                                        .withPosition(0, 1).withSize(3, 3)
                                        .withProperties(Map.of(
                                                        "Number of columns", 1,
                                                        "Number of rows", 1));
        public static final ShuffleboardLayout m_layoutLaunching = m_tabGeneral
                        .getLayout("Launching", BuiltInLayouts.kGrid)
                        .withPosition(3, 1).withSize(3, 3)
                        .withProperties(Map.of("Number of columns", 1,
                                        "Number of rows", 3));
        public static final Map<String, Object> kPropertiesDistanceSensor =
                        Map.of("Min", RoboRIO.kMinimumReadingUltrasonic, "Max",
                                        RoboRIO.kMaximumReadingUltrasonic,
                                        "Center",
                                        RoboRIO.kMinimumReadingUltrasonic);
        public static final NetworkTableEntry m_entryDistanceSensor =
                        m_layoutLaunching.add("Distance Sensor Reading", 0.0)
                                        .withWidget(BuiltInWidgets.kNumberBar)
                                        .withProperties(kPropertiesDistanceSensor)
                                        .getEntry();
        public static final NetworkTableEntry m_entryDistanceTolerence =
                        m_layoutLaunching.addPersistent("Distance Tolerance", 1)
                                        .withWidget(BuiltInWidgets.kNumberSlider)
                                        .withProperties(Map.of("Min", 0.0,
                                                        "Max", 2.0,
                                                        "Block increment",
                                                        0.25))
                                        .getEntry();
        public static final ShuffleboardLayout m_layoutControlPanel =
                        m_tabGeneral.getLayout("Color Sensing",
                                        BuiltInLayouts.kGrid).withPosition(3, 4)
                                        .withSize(3, 2)
                                        .withProperties(Map.of(
                                                        "Number of columns", 2,
                                                        "Number of rows", 2));
        public static final NetworkTableEntry m_entryDetectedColor =
                        m_layoutControlPanel.add("Detected color", "N/A")
                                        .getEntry();
        public static final NetworkTableEntry m_entryConfidence =
                        m_layoutControlPanel.add("Confidence", 0).getEntry();
        public static final NetworkTableEntry m_entryTargetColor =
                        m_layoutControlPanel.add("Target Color", "N/A")
                                        .getEntry();
        public static final NetworkTableEntry m_entryTargetSpin =
                        m_layoutControlPanel.add("Target Spins", 0).getEntry();
        public static final ShuffleboardLayout m_layoutVision =
                        m_tabGeneral.getLayout("Vision", BuiltInLayouts.kGrid)
                                        .withPosition(6, 0).withSize(1, 1)
                                        .withProperties(Map.of(
                                                        "Number of columns", 1,
                                                        "Number of rows", 1));

        // Shuffleboard Tools
        public static final ShuffleboardTab m_tabTools =
                        Shuffleboard.getTab("Tools");
        public static final ShuffleboardLayout m_layoutLaunchingTools =
                        m_tabTools.getLayout("Launching Tools",
                                        BuiltInLayouts.kGrid).withPosition(0, 0)
                                        .withSize(4, 5)
                                        .withProperties(Map.of(
                                                        "Number of columns", 2,
                                                        "Number of rows", 1));
        public static final ShuffleboardLayout m_layoutProjectileMotionPred =
                        m_layoutLaunchingTools.getLayout(
                                        "Projectile Motion Prediction",
                                        BuiltInLayouts.kList).withSize(2, 5);
        public static final NetworkTableEntry m_entryHorizontalDistance =
                        m_layoutProjectileMotionPred.addPersistent(
                                        "Horizontal Distance (m)", 0)
                                        .getEntry();
        public static final NetworkTableEntry m_entryRunPred =
                        m_layoutProjectileMotionPred.add("Calculate", false)
                                        .withWidget(BuiltInWidgets.kToggleButton)
                                        .getEntry();
        public static final NetworkTableEntry m_entryVerticalDistance =
                        m_layoutProjectileMotionPred
                                        .add("Vertical Distance (m)", 0)
                                        .withWidget(BuiltInWidgets.kTextView)
                                        .getEntry();
        public static final ShuffleboardLayout m_layoutProjectileMotionSim =
                        m_layoutLaunchingTools.getLayout(
                                        "Projectile Motion Simulation",
                                        BuiltInLayouts.kList).withSize(2, 5);
        public static final NetworkTableEntry m_entryRunSim =
                        m_layoutProjectileMotionSim.add("Run Simulation", false)
                                        .withWidget(BuiltInWidgets.kToggleButton)
                                        .getEntry();
        public static final NetworkTableEntry m_entrySimGraph =
                        m_layoutProjectileMotionSim
                                        .add("Simlulation", new double[] {0, 0})
                                        .withWidget(BuiltInWidgets.kGraph)
                                        .withProperties(Map.of("Visible time",
                                                        7))
                                        .getEntry();
        public static final NetworkTableEntry m_entrySimTime =
                        m_layoutProjectileMotionSim
                                        .add("Simulation Time (s)", 0)
                                        .withWidget(BuiltInWidgets.kTextView)
                                        .getEntry();
        public static final Timer m_timerSim = new Timer();
        public static boolean m_isRunningSim = false;

        /**
         * This function is called upon in the robotPeriodic() method of Robot.java.
         */
        public static void shuffleboardPeriodic() {
                if (ShuffleboardClass.m_entryRunPred.getBoolean(false)) {
                        ShuffleboardClass.m_entryRunPred.setBoolean(false);
                        double horDistance =
                                        ShuffleboardClass.m_entryHorizontalDistance
                                                        .getDouble(0);
                        // This expression calculates how high the ball will be at a specified distance
                        // away from the robot. See the research document for the derivation of the
                        // formula used here.
                        ShuffleboardClass.m_entryVerticalDistance
                                        .setDouble(horDistance * Math.tan(
                                                        Constants.kLauncherAngle)
                                                        - (0.5 * Constants.kAccelDueToGravity
                                                                        * Math.pow((horDistance
                                                                                        / (Constants.kInitialVelocityBall
                                                                                                        * Math.cos(Constants.kLauncherAngle))),
                                                                                        2)));
                }

                if (ShuffleboardClass.m_entryRunSim.getBoolean(false)
                                && !ShuffleboardClass.m_isRunningSim) {
                        ShuffleboardClass.m_isRunningSim = true;
                        ShuffleboardClass.m_timerSim.start();
                } else if (ShuffleboardClass.m_entryRunSim.getBoolean(false)
                                && ShuffleboardClass.m_isRunningSim) {
                        double time = ShuffleboardClass.m_timerSim.get();
                        double horizontalDistance =
                                        (Constants.kInitialVelocityBall * Math
                                                        .cos(Constants.kLauncherAngle))
                                                        * time;
                        double verticalDistance =
                                        (Constants.kInitialVelocityBall * Math
                                                        .sin(Constants.kLauncherAngle))
                                                        * time
                                                        + 0.5 * -Constants.kAccelDueToGravity
                                                                        * Math.pow(time, 2);
                        if (verticalDistance < 0) {
                                ShuffleboardClass.m_isRunningSim = false;
                                ShuffleboardClass.m_entryRunSim
                                                .setBoolean(false);
                                ShuffleboardClass.m_timerSim.reset();
                        } else {
                                ShuffleboardClass.m_entrySimGraph
                                                .setDoubleArray(new double[] {
                                                                horizontalDistance,
                                                                verticalDistance});
                                ShuffleboardClass.m_entrySimTime
                                                .setDouble(time);
                        }
                } else if (!ShuffleboardClass.m_entryRunSim.getBoolean(false)
                                && ShuffleboardClass.m_isRunningSim) {
                        // Cancel a running simulation.
                        ShuffleboardClass.m_isRunningSim = false;
                        ShuffleboardClass.m_timerSim.reset();
                        ShuffleboardClass.m_entrySimGraph
                                        .setDoubleArray(new double[] {0, 0});
                }
        }
}
