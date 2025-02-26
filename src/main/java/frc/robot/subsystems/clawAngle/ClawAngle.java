package frc.robot.subsystems.clawAngle;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ClawAngle extends SubsystemBase {
  private ClawAngleIO m_ClawAngleIO;

  ClawAngleIOInputsAutoLogged loggedclawangle = new ClawAngleIOInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder("ClawAngle", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public ClawAngle(ClawAngleIO clawAngleIO) {
    m_ClawAngleIO = clawAngleIO;
    loggedclawangle.clawAngle = Degrees.mutable(0);
    loggedclawangle.clawAngularVelocity = DegreesPerSecond.mutable(0);
    loggedclawangle.clawAngleSetPoint = Degrees.mutable(0);
    loggedclawangle.supplyCurrent = Amps.mutable(0);
    loggedclawangle.torqueCurrent = Amps.mutable(0);
    loggedclawangle.voltageSetPoint = Volts.mutable(0);
    loggedclawangle.voltage = Volts.mutable(0);

    this.m_ClawAngleIO.setGains(tunableGains.build());

    RobotState.instance().setClawAngleSource(loggedclawangle.clawAngle);
  }

  public Supplier<Angle> getAngleSupplier() {
    return () -> loggedclawangle.clawAngle;
  }

  public void setAngle(Angle angle) {
    m_ClawAngleIO.setTarget(angle);
  }

  public Command getNewClawAngleTurnCommand(DoubleSupplier angle) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(angle.getAsDouble()));
        },
        this);
  }

  public Command getNewApplyCoastModeCommand() {
    return new InstantCommand(
        () -> {
          m_ClawAngleIO.applyCoastMode();
        },
        this);
  }

  public Command getNewClawAngleTurnCommand(double i) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(i));
        },
        this);
  }

  /**
   * Returns when this joint is greater than 'angle' away from the forward horizontal
   *
   * @param angle
   * @return
   */
  public Trigger getNewGreaterThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(
        () -> {
          return loggedclawangle.clawAngle.in(Degrees) > angle.getAsDouble();
        });
  }

  public Trigger getNewAtAngleTrigger(Angle angle, Angle tolerance) {
    return new Trigger(
        () -> {
          return MathUtil.isNear(
              angle.baseUnitMagnitude(),
              loggedclawangle.clawAngle.baseUnitMagnitude(),
              tolerance.baseUnitMagnitude());
        });
  }

  public Trigger getNewAtSetpointTrigger() {
    return new Trigger(
        () -> {
          return MathUtil.isNear(
              loggedclawangle.clawAngleSetPoint.baseUnitMagnitude(),
              loggedclawangle.clawAngle.baseUnitMagnitude(),
              Degrees.of(0.25).baseUnitMagnitude());
        });
  }

  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_ClawAngleIO.setGains(gains));
    m_ClawAngleIO.updateInputs(loggedclawangle);
    Logger.processInputs("RobotState/ClawAngle", loggedclawangle);
  }
}
