// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleFunction;

/** Pose selection shared by every Tusk consumer. Mirrors {@code tusk.select_pose} in RobotPy. */
public final class Tusk {
  private Tusk() {}

  public static final double MAX_MEASUREMENT_AGE_SECONDS = 1.0;
  public static final double FUTURE_TOLERANCE_SECONDS = 0.02;

  private static boolean finitePose(Pose3d pose) {
    if (pose == null) return false;
    var q = pose.getRotation().getQuaternion();
    double norm =
        q.getW() * q.getW() + q.getX() * q.getX() + q.getY() * q.getY() + q.getZ() * q.getZ();
    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getZ())
        && Double.isFinite(norm)
        && Math.abs(norm - 1) < 1e-6;
  }

  public static boolean validObservation(VisionObservation obs) {
    return obs != null
        && obs.cameraId() >= 0
        && obs.tagCount() >= 1
        && obs.tagCount() <= 64
        && Double.isFinite(obs.ageSeconds())
        && obs.ageSeconds() >= 0
        && obs.ageSeconds() <= MAX_MEASUREMENT_AGE_SECONDS
        && positiveFinite(obs.stdDevFactor())
        && positiveFinite(obs.avgTagDistance())
        && Double.isFinite(obs.ambiguity())
        && obs.ambiguity() >= 0
        && obs.ambiguity() <= 1
        && finitePose(obs.robotPose())
        && finitePose(obs.altRobotPose());
  }

  public static boolean validCaptureTime(double captureTime, double now) {
    return Double.isFinite(captureTime)
        && Double.isFinite(now)
        && captureTime >= 0
        && now - captureTime >= -FUTURE_TOLERANCE_SECONDS
        && now - captureTime <= MAX_MEASUREMENT_AGE_SECONDS;
  }

  private static boolean positiveFinite(double v) {
    return Double.isFinite(v) && v > 0;
  }

  /** Throws for invalid confidence rather than letting a bad dashboard edit corrupt odometry. */
  public static Matrix<N3, N1> stdDevs(
      VisionObservation obs,
      double xyCoefficient,
      double thetaCoefficient,
      boolean trustSingleTheta) {
    if (!positiveFinite(obs.stdDevFactor())
        || !positiveFinite(xyCoefficient)
        || !positiveFinite(thetaCoefficient)) {
      throw new IllegalArgumentException(
          "vision confidence and coefficients must be finite and positive");
    }
    double xy = xyCoefficient * obs.stdDevFactor();
    double theta = thetaCoefficient * obs.stdDevFactor();
    if (!positiveFinite(xy) || !positiveFinite(theta)) {
      throw new IllegalArgumentException("vision standard deviations overflowed or underflowed");
    }
    if (obs.tagCount() == 1 && !trustSingleTheta) theta = Double.POSITIVE_INFINITY;
    return VecBuilder.fill(xy, xy, theta);
  }

  /**
   * Select and gate using the estimator's capture-time history. Missing history drops enabled
   * frames.
   */
  public static Optional<Pose2d> selectTimedPose(
      VisionObservation obs,
      double captureTime,
      double now,
      DoubleFunction<Optional<Pose2d>> sampleAt,
      Pose2d currentEstimate,
      boolean enabled,
      OptionalDouble maxSingleJump,
      OptionalDouble maxYaw,
      OptionalDouble maxMultiJump) {
    if (!validObservation(obs) || !validCaptureTime(captureTime, now)) return Optional.empty();
    var reference = sampleAt.apply(captureTime);
    if (reference.isEmpty() && !enabled) reference = Optional.ofNullable(currentEstimate);
    if (reference.isEmpty()) return Optional.empty();
    Pose2d ref = reference.get();
    if (!Double.isFinite(ref.getX())
        || !Double.isFinite(ref.getY())
        || !Double.isFinite(ref.getRotation().getRadians())) return Optional.empty();
    var pose =
        selectValidatedPose(obs, ref.getRotation(), enabled ? maxYaw : OptionalDouble.empty());
    OptionalDouble limit = obs.tagCount() == 1 ? maxSingleJump : maxMultiJump;
    if (enabled
        && pose.isPresent()
        && limit.isPresent()
        && pose.get().getTranslation().getDistance(ref.getTranslation()) > limit.getAsDouble()) {
      return Optional.empty();
    }
    return pose;
  }

  /**
   * The pose to use from one observation, or empty to drop it.
   *
   * <p>Multi-tag: robotPose. Single tag: whichever candidate's heading is closest to {@code
   * heading} (the estimator heading at image capture time), dropped if it's still more than {@code
   * maxYawErrorRad} off.
   *
   * @param maxYawErrorRad empty = never drop
   */
  public static Optional<Pose2d> selectPose(
      VisionObservation obs, Rotation2d heading, OptionalDouble maxYawErrorRad) {
    if (!validObservation(obs) || !Double.isFinite(heading.getRadians())) return Optional.empty();
    return selectValidatedPose(obs, heading, maxYawErrorRad);
  }

  private static Optional<Pose2d> selectValidatedPose(
      VisionObservation obs, Rotation2d heading, OptionalDouble maxYawErrorRad) {
    Pose2d best = obs.robotPose().toPose2d();
    if (obs.tagCount() != 1) {
      return Optional.of(best);
    }
    Pose2d alt = obs.altRobotPose().toPose2d();
    double errBest = Math.abs(best.getRotation().minus(heading).getRadians());
    double errAlt = Math.abs(alt.getRotation().minus(heading).getRadians());
    Pose2d pose = errBest <= errAlt ? best : alt;
    double err = Math.min(errBest, errAlt);
    if (maxYawErrorRad.isPresent() && err > maxYawErrorRad.getAsDouble()) {
      return Optional.empty();
    }
    return Optional.of(pose);
  }

  /** {@link #selectPose(VisionObservation, Rotation2d, OptionalDouble)} that never drops. */
  public static Optional<Pose2d> selectPose(VisionObservation obs, Rotation2d heading) {
    return selectPose(obs, heading, OptionalDouble.empty());
  }
}
