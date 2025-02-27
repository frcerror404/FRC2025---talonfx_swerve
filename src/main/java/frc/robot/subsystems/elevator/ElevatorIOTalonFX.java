package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  public MotionMagicVoltage Request;
  public TalonFX leaderMotor;
  public TalonFX followerMotor;

  public ElevatorIOInputs inputs;

  private Distance m_setPoint = Distance.ofBaseUnits(0, Inches);

  public ElevatorIOTalonFX(CanDef leftDef, CanDef rightDef) {
    leaderMotor = new TalonFX(leftDef.id(), leftDef.bus());
    followerMotor = new TalonFX(rightDef.id(), rightDef.bus());
    Request = new MotionMagicVoltage(0);

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration leaderCfg = new TalonFXConfiguration();
    leaderCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderCfg.Voltage.PeakForwardVoltage = 12;
    leaderCfg.Voltage.PeakReverseVoltage = 12;
    leaderCfg.CurrentLimits.StatorCurrentLimit = 120;
    leaderCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderCfg.CurrentLimits.SupplyCurrentLimit = 20;
    leaderCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 25;
    leaderCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leaderCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = .5;
    leaderCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leaderCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(leaderCfg));

    TalonFXConfiguration followerCfg = new TalonFXConfiguration();
    followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerCfg.Voltage.PeakForwardVoltage = 12;
    followerCfg.Voltage.PeakReverseVoltage = 12;
    followerCfg.CurrentLimits.StatorCurrentLimit = 120;
    followerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    followerCfg.CurrentLimits.SupplyCurrentLimit = 20;
    followerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    // config.Feedback.SensorToMechanismRatio = Elevator.REDUCTION;
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(followerCfg));

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double rotations = leaderMotor.getPosition().getValue().in(Rotations);
    inputs.distance.mut_replace(Inches.of(rotations * Elevator.INCHES_PER_ROT));
    inputs.velocity.mut_replace(
        InchesPerSecond.of(leaderMotor.getVelocity().getValue().in(RotationsPerSecond)));
    inputs.setPoint.mut_replace(m_setPoint);
    inputs.supplyCurrent.mut_replace(leaderMotor.getSupplyCurrent().getValue());
    inputs.voltage.mut_replace(leaderMotor.getMotorVoltage().getValue());
  }

  @Override
  public void setTarget(Distance target) {
    Request = Request.withPosition(target.in(Inches) / Elevator.INCHES_PER_ROT).withSlot(0);
    leaderMotor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void stop() {
    leaderMotor.setControl(new StaticBrake());
  }

  @Override
  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
