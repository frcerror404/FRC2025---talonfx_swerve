package frc.robot.subsystems.clawAngle;

import static frc.robot.subsystems.clawAngle.ClawAngleConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClawAngleIOTalonFX implements ClawAngleIO {
  private final TalonFX clawAngle = new TalonFX(clawAngleMotorID);
  private final StatusSignal<Angle> positionRot = clawAngle.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = clawAngle.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = clawAngle.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = clawAngle.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ClawAngleIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> clawAngle.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    clawAngle.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClawAngleIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    clawAngle.setControl(voltageRequest.withOutput(volts));
  }
}
