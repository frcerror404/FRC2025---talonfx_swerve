package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.clawAngle.ClawAngle;
import frc.robot.subsystems.elevator.Elevator;

public class StowHomeStopIntake extends SequentialCommandGroup {

  public StowHomeStopIntake(Elevator elevator, ClawAngle clawAngle, Claw claw) {
    super(
        new ClawAngleAvoidElevator(clawAngle),
        new ElevatorHome(elevator),
        new ClawAngleHome(clawAngle),
        new StopIntake(claw));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, clawAngle, claw);
  }
}
