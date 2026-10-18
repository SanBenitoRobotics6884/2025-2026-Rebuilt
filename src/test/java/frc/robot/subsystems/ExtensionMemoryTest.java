// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import frc.robot.subsystems.ExtensionMemory.Snapshot;

class ExtensionMemoryTest {
  private static final double kEpsilon = 1e-9;

  @TempDir Path tempDir;

  @Test
  void roundTripKeepsTheSweepAndThePosition() {
    Path file = tempDir.resolve("extension.txt");
    Snapshot written = new Snapshot(1.0, 69.0, 12.5);

    assertTrue(ExtensionMemory.write(file, written));

    Snapshot read = ExtensionMemory.read(file).orElseThrow();
    assertEquals(1.0, read.retractRotations, kEpsilon);
    assertEquals(69.0, read.extendRotations, kEpsilon);
    assertEquals(12.5, read.positionRotations, kEpsilon);
  }

  @Test
  void negativeTravelRoundTrips() {
    Path file = tempDir.resolve("extension.txt");
    assertTrue(ExtensionMemory.write(file, new Snapshot(-1.0, -39.0, -20.0)));

    Snapshot read = ExtensionMemory.read(file).orElseThrow();
    assertEquals(-1.0, read.retractRotations, kEpsilon);
    assertEquals(-39.0, read.extendRotations, kEpsilon);
    assertEquals(-20.0, read.positionRotations, kEpsilon);
  }

  @Test
  void missingFileIsEmpty() {
    assertTrue(ExtensionMemory.read(tempDir.resolve("missing.txt")).isEmpty());
  }

  @Test
  void shortTravelIsRejected() {
    Path file = tempDir.resolve("extension.txt");
    double minSpan = ExtensionMemory.minimumSpanRotations();
    assertTrue(ExtensionMemory.write(file, new Snapshot(0.0, minSpan / 2.0, 0.0)));

    assertTrue(ExtensionMemory.read(file).isEmpty());
  }

  @Test
  void corruptFileIsRejected() {
    Path file = tempDir.resolve("extension.txt");
    assertTrue(ExtensionMemory.write(file, new Snapshot(1.0, 69.0, 1.0)));
    try {
      Files.writeString(file, "version=1\nretractRotations=nope\n");
    } catch (Exception exception) {
      throw new RuntimeException(exception);
    }

    assertTrue(ExtensionMemory.read(file).isEmpty());
  }

  @Test
  void rewriteReplacesThePreviousPosition() {
    Path file = tempDir.resolve("extension.txt");
    assertTrue(ExtensionMemory.write(file, new Snapshot(1.0, 69.0, 1.0)));
    assertTrue(ExtensionMemory.write(file, new Snapshot(1.0, 69.0, 69.0)));

    Snapshot read = ExtensionMemory.read(file).orElseThrow();
    assertEquals(69.0, read.positionRotations, kEpsilon);
  }

  @Test
  void deleteRemovesTheSnapshot() throws Exception {
    Path file = tempDir.resolve("extension.txt");
    assertTrue(ExtensionMemory.write(file, new Snapshot(1.0, 69.0, 4.0)));

    ExtensionMemory.delete(file);

    assertFalse(Files.exists(file));
    assertTrue(ExtensionMemory.read(file).isEmpty());
  }

  @Test
  void nonFiniteSnapshotIsNotWritten() {
    Path file = tempDir.resolve("extension.txt");

    assertFalse(ExtensionMemory.write(file, new Snapshot(1.0, 69.0, Double.NaN)));
    assertFalse(Files.exists(file));
  }
}
