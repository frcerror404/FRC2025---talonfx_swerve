package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.clawAngle.ClawAngle;
import frc.robot.subsystems.elevator.Elevator;

public class StowHome extends SequentialCommandGroup {

  public StowHome(Elevator elevator, ClawAngle clawAngle) {
    super(
        new ClawAngleAvoidElevator(clawAngle),
        new ElevatorHome(elevator),
        new ClawAngleHome(clawAngle));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, clawAngle);
  }
}
