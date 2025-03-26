package frc.robot.subsystems.vision.alignment;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RotationUtil;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

public class Alignment extends SubsystemBase {
  private final Supplier<Rotation2d> gyro;
  private final VisionConsumer consumer;
  private final AlignmentIO[] io;
  private final AlignmentIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private List<Pose3d> allPoses;

  public Alignment(Supplier<Rotation2d> gyro, VisionConsumer consumer, AlignmentIO... io) {
    this.gyro = gyro;
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new AlignmentIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AlignmentIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with tracking.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (AlignmentIO.PoseObservation observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean acceptPose = shouldAccept(observation);

        // Add pose to log
        robotPoses.add(observation.pose());
        if (acceptPose) {
          robotPosesAccepted.add(observation.pose());
        } else {
          robotPosesRejected.add(observation.pose());
        }

        // Skip if rejected
        if (!acceptPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();

        // Send tracking observation
        consumer.accept(
            observation.pose().toPose2d(), observation.timestamp(), VecBuilder.fill(0, 0, 0));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    allPoses = allRobotPoses;
  }

  private boolean shouldAccept(AlignmentIO.PoseObservation observation) {
    if (observation.tagCount() == 0) {
      return false;
    }
    if (!RotationUtil.within(
        observation.pose().getRotation().toRotation2d(), gyro.get(), Rotation2d.kZero)) {
      return false;
    }
    if (Math.abs(observation.pose().getZ()) > 5) {
      return false; // Must have realistic Z coordinate
    }
    return (observation.pose().getX() >= 0.0)
        && (observation.pose().getX() <= aprilTagLayout.getFieldLength())
        && (observation.pose().getY() >= 0.0)
        && (observation.pose().getY()
            <= aprilTagLayout.getFieldWidth()); // Must be within the field boundaries
  }

  @Nullable
  public Pose3d getPose(int index) {
    if (index < allPoses.size()) {
      return allPoses.get(index);
    }
    return null;
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
