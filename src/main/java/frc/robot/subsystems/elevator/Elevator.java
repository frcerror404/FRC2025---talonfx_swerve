package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO m_ElevatorIO;

  ElevatorIOInputsAutoLogged loggedelevator = new ElevatorIOInputsAutoLogged();

  public static final double SPOOL_RADIUS = 1.751 / 2.0;

  public static final double INCHES_PER_ROT = (Math.PI * SPOOL_RADIUS * .85);

  public static final double REDUCTION = (5.0 / 1.0); // 5:1 Reduction

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder("Elevator", 8.0, 2, 0, 0, 0.72, 0, 0, 65.0, 90.0, 0, 0, 0);

  public Elevator(ElevatorIO ElevatorIO) {
    m_ElevatorIO = ElevatorIO;
    loggedelevator.distance = Inches.mutable(0);
    loggedelevator.velocity = InchesPerSecond.mutable(0);
    loggedelevator.setPoint = Meters.mutable(0);
    loggedelevator.supplyCurrent = Amps.mutable(0);
    loggedelevator.torqueCurrent = Amps.mutable(0);
    loggedelevator.voltageSetPoint = Volts.mutable(0);
    loggedelevator.voltage = Volts.mutable(0);

    this.m_ElevatorIO.setGains(tunableGains.build());
    RobotState.instance().setElevatorSource(loggedelevator.distance);
  }

  public Supplier<Distance> getDistanceExtendedSupplier() {
    return () -> loggedelevator.distance;
  }

  public void setDistance(Distance target) {
    m_ElevatorIO.setTarget(target);
  }

  public void setDistance(Distance target, boolean isSlow) {
    m_ElevatorIO.setTarget(target, isSlow);
  }

  public Command getNewSetDistanceCommand(DoubleSupplier distance) {
    return new InstantCommand(
        () -> {
          setDistance(Inches.of(distance.getAsDouble()));
        },
        this);
  }
  /**
   * @param i Inches
   */
  public Command getNewSetDistanceCommand(double i) {
    return new InstantCommand(
        () -> {
          setDistance(Inches.of(i));
        },
        this);
  }

  public Trigger getNewAtAngleTrigger(Distance dist, Distance tolerance) {
    return new Trigger(
        () -> {
          return MathUtil.isNear(
              dist.baseUnitMagnitude(),
              loggedelevator.distance.baseUnitMagnitude(),
              tolerance.baseUnitMagnitude());
        });
  }

  public boolean isElevatorAtDistance(Distance dist, Distance tolerance) {
    return MathUtil.isNear(
        dist.baseUnitMagnitude(),
        loggedelevator.distance.baseUnitMagnitude(),
        tolerance.baseUnitMagnitude());
  }

  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_ElevatorIO.setGains(gains));
    m_ElevatorIO.updateInputs(loggedelevator);
    Logger.processInputs("RobotState/Elevator", loggedelevator);
  }
}
