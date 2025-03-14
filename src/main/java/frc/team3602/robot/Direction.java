/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

public enum Direction {
  Left,
  Right;

  public Direction invert() {
    return (this == Direction.Left) ? Direction.Right : Direction.Left;
  }
}