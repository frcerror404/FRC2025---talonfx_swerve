package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class IntakeCoral extends Command {
  private Claw m_claw;

  public IntakeCoral(Claw claw) {
    m_claw = claw;

    addRequirements(claw);
  }

  public void initialize() {
    m_claw.setTarget(Volts.of(4));
  }

  public boolean isFinished() {
    return true;
  }
}
