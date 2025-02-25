package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;

        public MutAngle position = Degrees.mutable(0);
        public MutAngularVelocity velocity = DegreesPerSecond.mutable(0);

        public MutVoltage appliedVoltsLeader = Volts.mutable(0);
        public MutVoltage appliedVoltsFollower = Volts.mutable(0);

        public MutCurrent supplyCurrentLeader = Amps.mutable(0);
        public MutCurrent supplyCurrentFollower = Amps.mutable(0);

        public MutCurrent torqueCurrentLeader = Amps.mutable(0);
        public MutCurrent torqueCurrentFollower = Amps.mutable(0);

        public MutTemperature temperatureLeader = Celsius.mutable(0);
        public MutTemperature temperatureFollower = Celsius.mutable(0);

        public MutAngle setpointPosition = Degrees.mutable(0);
        public MutAngularVelocity setpointVelocity = DegreesPerSecond.mutable(0);
  }

  // updates inputs of ClawMotor subsystem

  void updateInputs(ClawIOInputs inputs);

  default void runSetpoint(Angle position) {}

  default void runVolts(Voltage volts) {}

  default void runCurrent(Current current) {}

  default void setBrakeMode(boolean enabled) {}

  default void setPID(double p, double i, double d) {}

  default void setFF(double kS, double kG, double kV, double kA) {}

  default void stop() {}
}
