// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Constants.Intake.kHardStopBackoffRotations;
import static frc.robot.Constants.Constants.Intake.kMinTravelRotations;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Optional;

/**
 * The last intake-screw calibration, stored on the roboRIO so a later boot can seed the encoder.
 *
 * <p>TalonFX relative position comes back at 0 after a power cycle. The screws hold still while
 * unpowered, so the position written here is where the mechanism was left, in the same frame as
 * the extend and retract setpoints (rotations from the fully-in hard stop).
 */
public final class ExtensionMemory {
  static final int VERSION = 1;

  private ExtensionMemory() {}

  /** One saved sweep. Positions are left-screw rotations. */
  public static final class Snapshot {
    public final double retractRotations;
    public final double extendRotations;
    public final double positionRotations;

    public Snapshot(double retractRotations, double extendRotations, double positionRotations) {
      this.retractRotations = retractRotations;
      this.extendRotations = extendRotations;
      this.positionRotations = positionRotations;
    }
  }

  /**
   * Reads a snapshot. An absent file, a partial write, or a travel shorter than a real sweep all
   * come back empty so the robot stays uncalibrated instead of trusting a bad number.
   */
  public static Optional<Snapshot> read(Path path) {
    if (path == null || !Files.isRegularFile(path)) {
      return Optional.empty();
    }
    Double retract = null;
    Double extend = null;
    Double position = null;
    Integer version = null;
    try (BufferedReader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
      String line;
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (line.isEmpty() || line.startsWith("#")) {
          continue;
        }
        int split = line.indexOf('=');
        if (split <= 0) {
          return Optional.empty();
        }
        String key = line.substring(0, split).trim();
        String raw = line.substring(split + 1).trim();
        switch (key) {
          case "version" -> version = Integer.valueOf(raw);
          case "retractRotations" -> retract = Double.valueOf(raw);
          case "extendRotations" -> extend = Double.valueOf(raw);
          case "positionRotations" -> position = Double.valueOf(raw);
          default -> {
            // Leave room for a later field without rejecting the file.
          }
        }
      }
    } catch (IOException | NumberFormatException exception) {
      return Optional.empty();
    }
    if (version == null || version.intValue() != VERSION) {
      return Optional.empty();
    }
    if (retract == null || extend == null || position == null) {
      return Optional.empty();
    }
    if (!Double.isFinite(retract) || !Double.isFinite(extend) || !Double.isFinite(position)) {
      return Optional.empty();
    }
    if (Math.abs(extend - retract) < minimumSpanRotations()) {
      return Optional.empty();
    }
    return Optional.of(new Snapshot(retract, extend, position));
  }

  /** Replaces {@code path}. A power loss mid-write leaves the previous file in place. */
  public static boolean write(Path path, Snapshot snapshot) {
    if (path == null || snapshot == null) {
      return false;
    }
    if (!Double.isFinite(snapshot.retractRotations)
        || !Double.isFinite(snapshot.extendRotations)
        || !Double.isFinite(snapshot.positionRotations)) {
      return false;
    }
    Path temp = path.resolveSibling(path.getFileName().toString() + ".tmp");
    try {
      Path parent = path.getParent();
      if (parent != null) {
        Files.createDirectories(parent);
      }
      Files.writeString(
          temp,
          "version="
              + VERSION
              + System.lineSeparator()
              + "retractRotations="
              + snapshot.retractRotations
              + System.lineSeparator()
              + "extendRotations="
              + snapshot.extendRotations
              + System.lineSeparator()
              + "positionRotations="
              + snapshot.positionRotations
              + System.lineSeparator(),
          StandardCharsets.UTF_8);
      moveIntoPlace(temp, path);
      return true;
    } catch (IOException exception) {
      try {
        Files.deleteIfExists(temp);
      } catch (IOException ignored) {
        // The next write replaces the temp file.
      }
      return false;
    }
  }

  /** Drops a saved sweep. Used when a new sweep starts and the old position is no longer true. */
  public static void delete(Path path) {
    if (path == null) {
      return;
    }
    try {
      Files.deleteIfExists(path);
      Files.deleteIfExists(path.resolveSibling(path.getFileName().toString() + ".tmp"));
    } catch (IOException exception) {
      // A leftover file is re-checked the next time a sweep tries to delete it.
    }
  }

  /**
   * Extend and retract sit {@code backoff} inside each hard stop, so the saved span is the
   * measured travel minus both backoffs.
   */
  static double minimumSpanRotations() {
    double span = kMinTravelRotations - 2.0 * kHardStopBackoffRotations;
    if (span > 0.0) {
      return span;
    }
    return Double.MIN_NORMAL;
  }

  private static void moveIntoPlace(Path temp, Path path) throws IOException {
    try {
      Files.move(temp, path, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
    } catch (AtomicMoveNotSupportedException exception) {
      Files.move(temp, path, StandardCopyOption.REPLACE_EXISTING);
    }
  }
}
