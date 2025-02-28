package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(Claw claw) {
    super(claw.getNewSetVoltsCommand(6));
    addRequirements(claw);
  }
}
