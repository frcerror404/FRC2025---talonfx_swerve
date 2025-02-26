package frc.robot.subsystems.clawAngle;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface ClawAngleIO {

  @AutoLog
  public static class ClawAngleIOInputs {
    public MutAngle clawAngle;
    public MutAngularVelocity clawAngularVelocity;
    public MutAngle clawAngleSetPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Angle target);

  public void applyCoastMode();

  public void updateInputs(ClawAngleIOInputs inputs);

  public void setGains(Gains gains);
}
