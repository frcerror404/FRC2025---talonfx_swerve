package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorL3Intake extends ParallelCommandGroup {
  public ElevatorL3Intake(Elevator elevator, Claw claw) {

    // Add your commands in the super() call, e.g.
    super(new ElevatorAlgaeL3(elevator), new IntakeAlgae(claw));
    // super(new FooCommand(), new BarCommand());
  }
}
