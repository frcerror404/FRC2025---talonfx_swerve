package frc.robot.subsystems.claw;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.LoggedTunableNumber;

public class ClawIOSim implements ClawIO {

  private Voltage appliedVoltage = Volts.mutable(0);

  private final FlywheelSim sim;
  private LoggedTunableNumber intakesensorDistance;

  public ClawIOSim(int motorId) {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
    intakesensorDistance = new LoggedTunableNumber("RobotState/Claw/setSensorInputInches", 2.0);
  }

  @Override
  public void setTarget(Voltage target) {
    runVolts(target);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void updateInputs(ClawIOInputs input) {
    input.angularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);
    input.sensorDistance.mut_replace(Inches.of(intakesensorDistance.get()));

    // Periodic
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  public void stop() {
    setTarget(Volts.of(0));
  }

  @Override
  public Distance getDistance() {
    return Inches.of(intakesensorDistance.get());
  }
}
