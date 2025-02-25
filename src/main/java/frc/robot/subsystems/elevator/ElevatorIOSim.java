package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotorSim sim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60Foc(2), 0.004, .25
    ),
    DCMotor.getKrakenX60Foc(2)
  );

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPosition(double positionRad) {
    positionRad = Units.rotationsToRadians(positionRad);;
  }
}
