// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.OptionalDouble;

/**
 * Reads Owl's batched observations: one NT4 struct-array read per loop for every camera. Mirrors
 * {@code tusk.TuskSource} (BatchedVisionSource) in RobotPy.
 *
 * <ul>
 *   <li>{@link #poll()} returns nothing for the first {@code autoIgnoreSeconds} of auto, so the
 *       starting pose set in autonomousInit isn't yanked by a noisy first frame.
 *   <li>After the robot has been disabled for {@code throttleAfterSeconds}, it asks Owl to drop to
 *       {@code throttleFps}. Enabling restores full rate.
 * </ul>
 */
public class TuskSource {
  /** One observation with its capture time on the robot's FPGA clock. */
  public record TimedObservation(double captureTimeSeconds, VisionObservation observation) {}

  public static final String DEFAULT_TOPIC = "/Vision/observations";

  private final StructArraySubscriber<VisionObservation> sub;
  private final IntegerSubscriber heartbeat;
  private final DoublePublisher throttlePub;
  private final double autoIgnoreSeconds;
  private final OptionalDouble throttleAfterSeconds;
  private final double throttleFps;

  private Double autoStart = null;
  private double disabledSince = Timer.getTimestamp();

  /** Defaults: {@link #DEFAULT_TOPIC}, ignore 2 s of auto, throttle to 1 fps after 5 s disabled. */
  public TuskSource() {
    this(DEFAULT_TOPIC, NetworkTableInstance.getDefault(), 2.0, OptionalDouble.of(5.0), 1.0);
  }

  /**
   * @param autoIgnoreSeconds poll() returns nothing for this long after auto starts (0 disables)
   * @param throttleAfterSeconds throttle once disabled this long; empty never throttles
   */
  public TuskSource(
      String topic,
      NetworkTableInstance inst,
      double autoIgnoreSeconds,
      OptionalDouble throttleAfterSeconds,
      double throttleFps) {
    sub =
        inst.getStructArrayTopic(topic, VisionObservation.struct)
            .subscribe(new VisionObservation[] {}, WpilibCompat.queueOptions(32));
    String base = topic.contains("/") ? topic.substring(0, topic.lastIndexOf('/')) : "";
    if (base.isEmpty()) {
      base = "/Vision";
    }
    heartbeat = inst.getIntegerTopic(base + "/heartbeat").subscribe(0);
    // -1 = no limit. Owl treats a missing topic the same way.
    throttlePub = inst.getDoubleTopic(base + "/throttleFps").publish();
    this.autoIgnoreSeconds = autoIgnoreSeconds;
    this.throttleAfterSeconds = throttleAfterSeconds;
    this.throttleFps = throttleFps;
  }

  /** True if Owl has sent a heartbeat within {@code timeoutSeconds} (it beats once a second). */
  public boolean isConnected(double timeoutSeconds) {
    var hb = heartbeat.getAtomic();
    double age = Timer.getTimestamp() - WpilibCompat.ntSeconds(hb.timestamp);
    return hb.timestamp > 0 && age >= -Tusk.FUTURE_TOLERANCE_SECONDS && age < timeoutSeconds;
  }

  public boolean isConnected() {
    return isConnected(1.5);
  }

  /** Updates the throttle request; returns true while vision should be ignored. */
  private boolean updateModes() {
    double now = Timer.getTimestamp();
    if (RobotState.isEnabled()) {
      disabledSince = now;
    }
    boolean throttle =
        throttleAfterSeconds.isPresent()
            && now - disabledSince >= throttleAfterSeconds.getAsDouble();
    throttlePub.set(throttle ? throttleFps : -1.0);

    if (RobotState.isAutonomous() && RobotState.isEnabled()) {
      if (autoStart == null) {
        autoStart = now;
      }
      return now - autoStart < autoIgnoreSeconds;
    }
    autoStart = null;
    return false;
  }

  /**
   * All new observations from every camera since the last poll, oldest first. Empty during the auto
   * ignore window. Call it every loop, even when not using vision: it drains the queue and updates
   * the throttle request.
   */
  public List<TimedObservation> poll() {
    boolean ignore = updateModes();
    TimestampedObject<VisionObservation[]>[] entries = sub.readQueue(); // always drained
    List<TimedObservation> pending = new ArrayList<>();
    if (ignore) {
      return pending;
    }
    for (var entry : entries) {
      double msgTime =
          WpilibCompat.ntSeconds(
              entry.timestamp); // NT4 already converted this to the robot's clock
      for (VisionObservation obs : entry.value) {
        double captureTime = msgTime - obs.ageSeconds();
        if (Tusk.validObservation(obs)
            && Tusk.validCaptureTime(captureTime, Timer.getTimestamp())) {
          pending.add(new TimedObservation(captureTime, obs));
        }
      }
    }
    pending.sort(Comparator.comparingDouble(TimedObservation::captureTimeSeconds));
    return pending;
  }
}
