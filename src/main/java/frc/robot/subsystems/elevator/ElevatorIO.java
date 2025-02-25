package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

// Elevator has 2 motors!!
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorTemperature;
  }

  // updates inputs of Elevator subsystem

  public default void updateInputs(ElevatorIOInputs inputs) {}
  ;

  public default void setVelocity(double velocity) {}
  ;

  public default void setVoltage(double volts) {}
  ;

  public default void setPosition(double position) {}
  ;
}
