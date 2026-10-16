// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.neiacademy.robotics.frc2026.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Vision backend. true = Strix (Owl on the coprocessor publishes /Vision/observations, read
  // through VisionIOTusk); false = PhotonVision. Applies to REAL and SIM. In SIM, Tusk only sees
  // data if something publishes to NT (a real Owl pointed at the sim, or a fake Owl script).
  public static boolean useTusk = false;

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "frontcam"; // location
  //   public static String camera1Name = "rightcam"; // location`

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(-11.290),
          Units.inchesToMeters(8.061 + 7.5),
          Units.inchesToMeters(19.626),
          new Rotation3d(0, Math.toRadians(-67), 0));

  //   public static Transform3d robotToCamera1 =
  //       new Transform3d(
  //           Units.inchesToMeters(-11.2),
  //           Units.inchesToMeters(-2.8),
  //           Units.inchesToMeters(5),
  //           new Rotation3d(0.0, Math.toRadians(10), Math.toRadians(-110)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {1.0};

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
