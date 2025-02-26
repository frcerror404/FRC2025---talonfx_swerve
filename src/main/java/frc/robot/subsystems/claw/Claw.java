package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
  private ClawIO m_ClawIO;

  ClawIOInputsAutoLogged logged = new ClawIOInputsAutoLogged();

  public Claw(ClawIO ClawIO) {
    m_ClawIO = ClawIO;
    logged.angularVelocity = DegreesPerSecond.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.voltage = Volts.mutable(0);
    logged.sensorDistance = Meters.mutable(0);
  }

  public BooleanSupplier placeholderGetHasCoralSupplier() {
    return () -> false;
  }

  public void setTarget(Voltage target) {
    m_ClawIO.setTarget(target);
  }

  public Distance getDistance() {
    return m_ClawIO.getDistance();
  }

  public Command getNewSetVoltsCommand(LoggedTunableNumber volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of((volts.get())));
        },
        this);
  }

  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(i));
        },
        this);
  }

  @Override
  public void periodic() {
    m_ClawIO.updateInputs(logged);
    Logger.processInputs("RobotState/Claw", logged);
  }
}
