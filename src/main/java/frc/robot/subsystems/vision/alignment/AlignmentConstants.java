package frc.robot.subsystems.vision.alignment;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AlignmentConstants {

  public static final String ll = "limelight";
  public static final Transform3d llPos =
      new Transform3d(
          Units.inchesToMeters(9.5), // (35.0 / 2) - 8.0
          Units.inchesToMeters(11.5), // (25.0 / 2) - 1.0
          Units.inchesToMeters(6.25),
          new Rotation3d(0.0, 0.0, Math.PI / 2));
}
