package frc.robot.subsystems.clawAngle;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.Gains;

public class ClawAngleIOSim implements ClawAngleIO {

  private Voltage appliedVoltage = Volts.mutable(0.0);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(1000, 361));

  private final FlywheelSim sim;

  public ClawAngleIOSim(int motorId) {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500Foc(1), 0.28616, 1),
            DCMotor.getFalcon500Foc(1),
            new double[] {0.001});
  }

  @Override
  public void setTarget(Angle target) {
    controller.setGoal(new State(target.in(Degrees), 0));
  }

  private void updateVoltageSetpoint() {
    // FlywheelSim needs position
    Angle currentAngle = Radians.of(sim.getOutput(0));

    Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees)));

    Voltage effort = controllerVoltage;

    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void updateInputs(ClawAngleIOInputs input) {
    // update inputs
    input.clawAngle.mut_replace(Degrees.convertFrom(sim.getOutput(0), Radians), Degrees);
    input.clawAngularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.clawAngleSetPoint.mut_replace(controller.getGoal().position, Degrees);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);

    // Periodic
    updateVoltageSetpoint();
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  public void stop() {
    Angle currentAngle = Radians.of(sim.getOutput(0));
    controller.reset(currentAngle.in(Degrees));
    runVolts(Volts.of(0));
  }

  public void setGains(Gains gains) {
    DriverStation.reportWarning("Sim gains tuning not implemented", true);
  }

  @Override
  public void applyCoastMode() {
    System.out.println("Applied coast mode!");
  }
}
