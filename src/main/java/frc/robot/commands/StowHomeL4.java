package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.clawAngle.ClawAngle;
import frc.robot.subsystems.elevator.Elevator;

public class StowHomeL4 extends SequentialCommandGroup {

  public StowHomeL4(Elevator elevator, ClawAngle clawAngle) {
    super(new ElevatorL4(elevator), new ClawAngleHome(clawAngle));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, clawAngle);
  }
}
