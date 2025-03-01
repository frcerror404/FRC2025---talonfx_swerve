package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.clawAngle.ClawAngle;
import frc.robot.subsystems.elevator.Elevator;

public class PositionProcessor extends ParallelCommandGroup {
  public PositionProcessor(ClawAngle clawAngle, Elevator elevator, Claw claw) {

    // Add your commands in the super() call, e.g.
    super(new AlgaeRemoveAngle(clawAngle), new ElevatorProcessor(elevator), new IntakeAlgae(claw));
    // super(new FooCommand(), new BarCommand());
  }
}
