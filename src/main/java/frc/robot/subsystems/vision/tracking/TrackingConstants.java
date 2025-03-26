package frc.robot.subsystems.vision.tracking;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class TrackingConstants {

  public static final boolean useUnlikelyPVEstimates = false;

  // Camera names, must match names configured on coprocessor
  public static final String pv0c0 = "Cornelius";
  public static final String pv0c1 = "Bartholomew";

  private static final double frontModulePosition = Units.inchesToMeters((35.0 / 2) - 1.5);
  private static final double leftModulePosition = Units.inchesToMeters((25.0 / 2) - 1.5);
  private static final double upModulePosition = Units.inchesToMeters(7.5); // +z
  private static final double upModulePitch = -Math.PI / 6;

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d pv0c0Pos =
      new Transform3d(
          frontModulePosition,
          -leftModulePosition,
          upModulePosition,
          new Rotation3d(0.0, upModulePitch, 7 * Math.PI / 4));
  public static final Transform3d pv0c1Pos =
      new Transform3d(
          -frontModulePosition,
          leftModulePosition,
          upModulePosition,
          new Rotation3d(0.0, upModulePitch, 3 * Math.PI / 4));

  // Basic filtering thresholds
  public static double maxAmbiguity = 1.3;
  public static final double maxZError = 0.75;
  public static final Rotation2d maxYawError = Rotation2d.fromDegrees(5.0);

  // Standard deviation baselines, for 1-meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
