package frc.robot.subsystems.clawAngle;

import org.littletonrobotics.junction.AutoLog;

public interface ClawAngleIO {
  @AutoLog
  public static class ClawAngleIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorTemperature;
  }

  // Updates the inputs of ClawAngle
  public default void updateInputs(ClawAngleIOInputs inputs) {}
  ;

  // Sets speed of the motor
  public default void setVelocity(double velocity) {}
  ;

  public default void setVoltage(double volts) {}
  ;

  // Sets the target position of the motor
  public default void setPosition(double position) {}
  ;
}
