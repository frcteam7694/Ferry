package frc.robot.subsystems.vision;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.Robot;
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
      aprilTagLayout =
          Robot.isReal() ? getFieldFromFile(Path.of("/home/lvuser/deploy/layout.json")) : null;
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

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
