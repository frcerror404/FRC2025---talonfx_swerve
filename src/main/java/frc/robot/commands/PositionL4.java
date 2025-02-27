package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.clawAngle.ClawAngle;
import frc.robot.subsystems.elevator.Elevator;

public class PositionL4 extends ParallelCommandGroup {
  public PositionL4(ClawAngle clawAngle, Elevator elevator) {

    // Add your commands in the super() call, e.g.
    super(new ClawAngleAvoidElevator(clawAngle), new ElevatorL4(elevator));
    // super(new FooCommand(), new BarCommand());
  }
}
