package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class StopIntake extends Command {
  private Claw m_claw;

  public StopIntake(Claw claw) {
    m_claw = claw;

    addRequirements(claw);
  }

  public void initialize() {
    m_claw.setTarget(Volts.of(0));
  }

  public boolean isFinished() {
    return true;
  }
}
