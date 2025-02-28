package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorAlgaeL3 extends Command {
  private Elevator m_elevator;
  private Distance m_distance;

  public ElevatorAlgaeL3(Elevator elevator) {
    m_elevator = elevator;
    m_distance = Inches.of(40.0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setDistance(m_distance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isElevatorAtDistance(m_distance, Inches.of(2.0));
  }
}
