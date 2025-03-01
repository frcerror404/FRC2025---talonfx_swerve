package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.clawAngle.ClawAngle;
import frc.robot.subsystems.elevator.Elevator;

public class PositionL4 extends SequentialCommandGroup {
  public PositionL4(ClawAngle clawAngle, Elevator elevator) {
    // Add your commands in the super() call, e.g.
    super(
        new ParallelCommandGroup(new ClawAngleAvoidElevator(clawAngle), new ElevatorL4(elevator)),
        new L4Angle(clawAngle));

    addRequirements(clawAngle, elevator);
  }
}
