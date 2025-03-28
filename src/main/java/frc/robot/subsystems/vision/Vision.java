package frc.robot.subsystems.vision;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

public class Vision {

  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout;

  static {
    try {
      aprilTagLayout = getFieldFromString();
      //          Robot.isReal()
      //              ? getFieldFromFile(Path.of("/home/lvuser/deploy/layout.json"))
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  private static AprilTagFieldLayout getFieldFromString() throws IOException {
    String field =
        "{\"tags\":[{\"ID\":1,\"pose\":{\"translation\":{\"x\":16.697198,\"y\":0.65532,\"z\":1.4859},\"rotation\":{\"quaternion\":{\"W\":0.4539904997395468,\"X\":0,\"Y\":0,\"Z\":0.8910065241883678}}}},{\"ID\":2,\"pose\":{\"translation\":{\"x\":16.697198,\"y\":7.3964799999999995,\"z\":1.4859},\"rotation\":{\"quaternion\":{\"W\":-0.45399049973954675,\"X\":0,\"Y\":0,\"Z\":0.8910065241883679}}}},{\"ID\":3,\"pose\":{\"translation\":{\"x\":11.560809999999998,\"y\":8.05561,\"z\":1.30175},\"rotation\":{\"quaternion\":{\"W\":-0.7071067811865475,\"X\":0,\"Y\":0,\"Z\":0.7071067811865476}}}},{\"ID\":4,\"pose\":{\"translation\":{\"x\":9.276079999999999,\"y\":6.137656,\"z\":1.8679160000000001},\"rotation\":{\"quaternion\":{\"W\":0.9659258262890683,\"X\":0,\"Y\":0.25881904510252074,\"Z\":0}}}},{\"ID\":5,\"pose\":{\"translation\":{\"x\":9.276079999999999,\"y\":1.914906,\"z\":1.8679160000000001},\"rotation\":{\"quaternion\":{\"W\":0.9659258262890683,\"X\":0,\"Y\":0.25881904510252074,\"Z\":0}}}},{\"ID\":6,\"pose\":{\"translation\":{\"x\":13.474446,\"y\":3.3063179999999996,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":-0.8660254037844387,\"X\":0,\"Y\":0,\"Z\":0.49999999999999994}}}},{\"ID\":7,\"pose\":{\"translation\":{\"x\":13.890498,\"y\":4.0259,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":1,\"X\":0,\"Y\":0,\"Z\":0}}}},{\"ID\":8,\"pose\":{\"translation\":{\"x\":13.474446,\"y\":4.745482,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0,\"Y\":0,\"Z\":0.49999999999999994}}}},{\"ID\":9,\"pose\":{\"translation\":{\"x\":12.643358,\"y\":4.745482,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0,\"Y\":0,\"Z\":0.8660254037844386}}}},{\"ID\":10,\"pose\":{\"translation\":{\"x\":12.227305999999999,\"y\":4.0259,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766e-17,\"X\":0,\"Y\":0,\"Z\":1}}}},{\"ID\":11,\"pose\":{\"translation\":{\"x\":12.643358,\"y\":3.3063179999999996,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":-0.4999999999999998,\"X\":0,\"Y\":0,\"Z\":0.8660254037844387}}}},{\"ID\":12,\"pose\":{\"translation\":{\"x\":0.851154,\"y\":0.65532,\"z\":1.4859},\"rotation\":{\"quaternion\":{\"W\":0.8910065241883679,\"X\":0,\"Y\":0,\"Z\":0.45399049973954675}}}},{\"ID\":13,\"pose\":{\"translation\":{\"x\":0.851154,\"y\":7.3964799999999995,\"z\":1.4859},\"rotation\":{\"quaternion\":{\"W\":-0.8910065241883678,\"X\":0,\"Y\":0,\"Z\":0.45399049973954686}}}},{\"ID\":14,\"pose\":{\"translation\":{\"x\":8.272272,\"y\":6.137656,\"z\":1.8679160000000001},\"rotation\":{\"quaternion\":{\"W\":5.914589856893349e-17,\"X\":-0.25881904510252074,\"Y\":1.5848095757158825e-17,\"Z\":0.9659258262890683}}}},{\"ID\":15,\"pose\":{\"translation\":{\"x\":8.272272,\"y\":1.914906,\"z\":1.8679160000000001},\"rotation\":{\"quaternion\":{\"W\":5.914589856893349e-17,\"X\":-0.25881904510252074,\"Y\":1.5848095757158825e-17,\"Z\":0.9659258262890683}}}},{\"ID\":16,\"pose\":{\"translation\":{\"x\":5.9875419999999995,\"y\":-0.0038099999999999996,\"z\":1.30175},\"rotation\":{\"quaternion\":{\"W\":0.7071067811865476,\"X\":0,\"Y\":0,\"Z\":0.7071067811865476}}}},{\"ID\":17,\"pose\":{\"translation\":{\"x\":4.073905999999999,\"y\":3.3063179999999996,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":-0.4999999999999998,\"X\":0,\"Y\":0,\"Z\":0.8660254037844387}}}},{\"ID\":18,\"pose\":{\"translation\":{\"x\":3.6576,\"y\":4.0259,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766e-17,\"X\":0,\"Y\":0,\"Z\":1}}}},{\"ID\":19,\"pose\":{\"translation\":{\"x\":4.073905999999999,\"y\":4.745482,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0,\"Y\":0,\"Z\":0.8660254037844386}}}},{\"ID\":20,\"pose\":{\"translation\":{\"x\":4.904739999999999,\"y\":4.745482,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0,\"Y\":0,\"Z\":0.49999999999999994}}}},{\"ID\":21,\"pose\":{\"translation\":{\"x\":5.321046,\"y\":4.0259,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":1,\"X\":0,\"Y\":0,\"Z\":0}}}},{\"ID\":22,\"pose\":{\"translation\":{\"x\":4.904739999999999,\"y\":3.3063179999999996,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":-0.8660254037844387,\"X\":0,\"Y\":0,\"Z\":0.49999999999999994}}}}],\"field\":{\"length\":17.548,\"width\":8.052}}";

    InputStream stream = new ByteArrayInputStream(field.getBytes(StandardCharsets.UTF_8));
    InputStreamReader reader = new InputStreamReader(stream, StandardCharsets.UTF_8);

    try {
      return (AprilTagFieldLayout)
          (new ObjectMapper()).readerFor(AprilTagFieldLayout.class).readValue(reader);
    } catch (IOException ignored) {
      throw new IOException("Failed to load AprilTagFieldLayout from string.");
    }
  }

  private static AprilTagFieldLayout getFieldFromFile(Path resourcePath) throws IOException {
    InputStream stream = Files.newInputStream(resourcePath);
    InputStreamReader reader = new InputStreamReader(stream, StandardCharsets.UTF_8);

    try {
      return (AprilTagFieldLayout)
          (new ObjectMapper()).readerFor(AprilTagFieldLayout.class).readValue(reader);
    } catch (IOException ignored) {
      throw new IOException("Failed to load AprilTagFieldLayout: " + resourcePath);
    }
  }
}
