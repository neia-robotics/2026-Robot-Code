// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import edu.wpi.first.networktables.PubSubOption;

/** The few WPILib calls that differ between seasons by more than a package name (2026). */
final class WpilibCompat {
  private WpilibCompat() {}

  static double ntSeconds(long timestamp) {
    return timestamp / 1e6;
  }

  /** Subscriber options that queue every value (no dropped frames between polls). */
  static PubSubOption[] queueOptions(int pollStorage) {
    return new PubSubOption[] {
      PubSubOption.sendAll(true),
      PubSubOption.keepDuplicates(true),
      PubSubOption.pollStorage(pollStorage)
    };
  }
}
