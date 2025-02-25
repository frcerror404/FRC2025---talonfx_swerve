package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorTemperature;
  }

  // updates inputs of Climber subsystem
  public default void updateInputs(ClimberIOInputs inputs) {}
  ;

  public default void setVoltage(double volts) {}
  ;

  public default void setVelocity(double velocity) {}
  ;

  public default void setPosition(double position) {}
  ;
}
