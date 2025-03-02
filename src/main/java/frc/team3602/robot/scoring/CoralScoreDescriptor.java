package frc.team3602.robot.scoring;

public class CoralScoreDescriptor {
  
  private Direction direction;
  private double elevatorHeight;
  private double pivotAngle;

  public CoralScoreDescriptor(Direction direction, double elevatorHeight, double pivotAngle) {
    this.direction = direction;
    this.elevatorHeight = elevatorHeight;
    this.pivotAngle = pivotAngle;
  }

  public CoralScoreDescriptor invertDirection() {
    this.direction = (direction == Direction.Left) ? Direction.Right : Direction.Left;
    return this;
  }

  public Direction getDirection() {
    return this.direction;
  }

  public double getElevatorHeight() {
    return this.elevatorHeight;
  }

  public double getPivotAngle() {
    return this.pivotAngle;
  }

}