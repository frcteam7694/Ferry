// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout;

  static {
    try {
      aprilTagLayout = getFieldFromFile(Path.of("/home/lvuser/deploy/layout.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final boolean useUnlikelyPVEstimates = false;

  // Camera names, must match names configured on coprocessor
  public static final String pv1c1 = "pv1c1";
  public static final String pv1c2 = "pv1c2";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d pv1c1Pos =
      new Transform3d(0.3683, 0.0381, 0.0508, new Rotation3d(0.0, 0.0, Math.PI / 2));
  public static final Transform3d pv1c2Pos =
      new Transform3d(-.3429, -.0635, 0.4826, new Rotation3d(0.0, 0.0, -Math.PI / 2));

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

  private static AprilTagFieldLayout getFieldFromFile(Path resourcePath) throws IOException {
    InputStream stream = Files.newInputStream(resourcePath);
    InputStreamReader reader = new InputStreamReader(stream, StandardCharsets.UTF_8);

    try {
      return (AprilTagFieldLayout)
          (new ObjectMapper()).readerFor(AprilTagFieldLayout.class).readValue(reader);
    } catch (IOException var4) {
      throw new IOException("Failed to load AprilTagFieldLayout: " + resourcePath);
    }
  }
}
