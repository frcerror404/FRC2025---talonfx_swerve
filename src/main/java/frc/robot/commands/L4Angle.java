package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.clawAngle.ClawAngle;

public class L4Angle extends Command {
  private ClawAngle m_ClawAngle;
  private Angle m_angle;

  public L4Angle(ClawAngle clawAngle) {
    m_ClawAngle = clawAngle;
    m_angle = Degrees.of(-90);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ClawAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClawAngle.setAngle(m_angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ClawAngle.isClawAngleAtAngle(m_angle, Degrees.of(10));
  }
}
