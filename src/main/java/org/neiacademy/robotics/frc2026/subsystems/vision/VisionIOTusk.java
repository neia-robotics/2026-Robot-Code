// Adapted from Strix's tusk/java/examples/VisionIOTusk.java (MIT, NEIA Robotics 9644 and Choate
// Robotics 7407). The strix.tusk package is exported from the Strix repo with:
//   (cd strix/tusk/java && ./gradlew exportSources -PwpilibYear=2026)
// and copied from build/export/tusk-src-2026/. Re-export when Strix changes the struct.

package org.neiacademy.robotics.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import strix.tusk.Tusk;
import strix.tusk.TuskSource;
import strix.tusk.VisionObservation;

/**
 * Vision IO for Strix. Owl (the coprocessor) batches every camera into one NT topic, {@code
 * /Vision/observations}, so one VisionIOTusk covers all Owl cameras.
 *
 * <p>For single-tag frames Owl sends both IPPE candidates; Tusk picks the one whose heading matches
 * the pose estimator's heading at capture time. While enabled, frames that jump too far from the
 * estimate or disagree with the heading are dropped.
 */
public class VisionIOTusk implements VisionIO {
  // Gates applied while enabled (Tusk skips them while disabled so the robot can relocalize).
  private static final OptionalDouble maxSingleTagJumpMeters = OptionalDouble.of(2.0);
  private static final OptionalDouble maxSingleTagYawErrorRad =
      OptionalDouble.of(Math.toRadians(5.0));
  private static final OptionalDouble maxMultiTagJumpMeters = OptionalDouble.of(2.0);

  private final TuskSource source;
  private final DoubleFunction<Optional<Pose2d>> sampleAt;
  private final Supplier<Pose2d> currentPose;

  /**
   * @param source reader for Owl's NT topic
   * @param sampleAt the pose estimator's history lookup (timestamp in seconds)
   * @param currentPose the latest pose estimate, used when history is missing while disabled
   */
  public VisionIOTusk(
      TuskSource source, DoubleFunction<Optional<Pose2d>> sampleAt, Supplier<Pose2d> currentPose) {
    this.source = source;
    this.sampleAt = sampleAt;
    this.currentPose = currentPose;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = source.isConnected();

    List<PoseObservation> poses = new ArrayList<>();
    Set<Integer> tagIds = new LinkedHashSet<>();
    // poll() must run every loop: it drains the NT queue and updates Owl's throttle request.
    for (var timed : source.poll()) {
      VisionObservation obs = timed.observation();
      Optional<Pose2d> chosen =
          Tusk.selectTimedPose(
              obs,
              timed.captureTimeSeconds(),
              Timer.getTimestamp(),
              sampleAt,
              currentPose.get(),
              DriverStation.isEnabled(),
              maxSingleTagJumpMeters,
              maxSingleTagYawErrorRad,
              maxMultiTagJumpMeters);
      if (chosen.isEmpty()) {
        continue;
      }
      tagIds.addAll(obs.tagIds());
      poses.add(
          new PoseObservation(
              timed.captureTimeSeconds(),
              pickCandidate(obs, chosen.get()),
              obs.ambiguity(),
              obs.tagCount(),
              obs.avgTagDistance(),
              PoseObservationType.TUSK));
    }
    inputs.poseObservations = poses.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }

  /** The 3D candidate (robotPose or altRobotPose) that Tusk's 2D choice came from. */
  private static Pose3d pickCandidate(VisionObservation obs, Pose2d chosen) {
    double bestError = distance(obs.robotPose().toPose2d(), chosen);
    double altError = distance(obs.altRobotPose().toPose2d(), chosen);
    return bestError <= altError ? obs.robotPose() : obs.altRobotPose();
  }

  private static double distance(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation())
        + Math.abs(a.getRotation().minus(b.getRotation()).getRadians());
  }
}
