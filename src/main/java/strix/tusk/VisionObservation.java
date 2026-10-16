// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * One camera's robot-pose estimate from one frame, as published by Owl.
 *
 * <p>Must match {@code owl/vision_types.py} field for field; the struct name is versioned so a
 * robot and coprocessor on different versions see no data instead of misreading each other.
 *
 * @param cameraId index into the coprocessor's camera list
 * @param robotPose field-relative robot pose (WPILib blue-origin coordinates)
 * @param altRobotPose single tag: the other IPPE candidate; multi-tag: same as robotPose
 * @param ageSeconds publish time minus capture time
 * @param stdDevFactor avgTagDistance^2 / tagCount^2 * the camera's std_dev_factor
 * @param tagCount number of tags used
 * @param avgTagDistance meters, camera to tags
 * @param ambiguity single-tag best_error / alt_error; 0 for multi-tag
 * @param tagMask bit i set = tag ID i was used (IDs 0-63)
 */
public record VisionObservation(
    int cameraId,
    Pose3d robotPose,
    Pose3d altRobotPose,
    double ageSeconds,
    double stdDevFactor,
    int tagCount,
    double avgTagDistance,
    double ambiguity,
    long tagMask)
    implements StructSerializable {

  public static final VisionObservationStruct struct = new VisionObservationStruct();

  /** Tag IDs set in {@link #tagMask()}, in ascending order. */
  public List<Integer> tagIds() {
    List<Integer> ids = new ArrayList<>();
    for (int i = 0; i < 64; i++) {
      if ((tagMask & (1L << i)) != 0) {
        ids.add(i);
      }
    }
    return ids;
  }

  public static final class VisionObservationStruct implements Struct<VisionObservation> {
    // Struct.kSize* constants were removed in WPILib 2027.
    private static final int SIZE_INT32 = 4;
    private static final int SIZE_INT64 = 8;
    private static final int SIZE_DOUBLE = 8;

    @Override
    public Class<VisionObservation> getTypeClass() {
      return VisionObservation.class;
    }

    @Override
    public String getTypeName() {
      return "VisionObservationV3";
    }

    @Override
    public int getSize() {
      return SIZE_INT32
          + 2 * Pose3d.struct.getSize()
          + 2 * SIZE_DOUBLE
          + SIZE_INT32
          + 2 * SIZE_DOUBLE
          + SIZE_INT64;
    }

    @Override
    public String getSchema() {
      return "int32 cameraId; Pose3d robotPose; Pose3d altRobotPose; double ageSeconds; "
          + "double stdDevFactor; int32 tagCount; double avgTagDistance; double ambiguity; "
          + "int64 tagMask";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {Pose3d.struct};
    }

    @Override
    public VisionObservation unpack(ByteBuffer bb) {
      int cameraId = bb.getInt();
      Pose3d robotPose = Pose3d.struct.unpack(bb);
      Pose3d altRobotPose = Pose3d.struct.unpack(bb);
      double ageSeconds = bb.getDouble();
      double stdDevFactor = bb.getDouble();
      int tagCount = bb.getInt();
      double avgTagDistance = bb.getDouble();
      double ambiguity = bb.getDouble();
      long tagMask = bb.getLong();
      return new VisionObservation(
          cameraId,
          robotPose,
          altRobotPose,
          ageSeconds,
          stdDevFactor,
          tagCount,
          avgTagDistance,
          ambiguity,
          tagMask);
    }

    @Override
    public void pack(ByteBuffer bb, VisionObservation value) {
      bb.putInt(value.cameraId);
      Pose3d.struct.pack(bb, value.robotPose);
      Pose3d.struct.pack(bb, value.altRobotPose);
      bb.putDouble(value.ageSeconds);
      bb.putDouble(value.stdDevFactor);
      bb.putInt(value.tagCount);
      bb.putDouble(value.avgTagDistance);
      bb.putDouble(value.ambiguity);
      bb.putLong(value.tagMask);
    }

    @Override
    public boolean isImmutable() {
      return true;
    }
  }
}
