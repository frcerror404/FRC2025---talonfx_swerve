package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.clawAngle.ClawAngle;
import frc.robot.subsystems.elevator.Elevator;

public class StowProcessor extends SequentialCommandGroup {

  public StowProcessor(Elevator elevator, ClawAngle clawAngle) {
    super(new AlgaeRemoveAngle(clawAngle), new ElevatorProcessor(elevator));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, clawAngle);
  }
}
