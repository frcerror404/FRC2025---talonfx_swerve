package frc.robot.subsystems.clawAngle;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ClawAngleIOTalonFX implements ClawAngleIO {
  public MotionMagicVoltage Request;
  public CoastOut coastRequest;
  public TalonFX Motor;
  public CANcoder canCoder;
  public Angle canCoderOffset = Rotations.of(-0.487793);
  private Angle m_setPoint = Angle.ofRelativeUnits(.0, Rotations);

  public ClawAngleIOTalonFX(CanDef motorId, CanDef canCoderId) {
    Motor = new TalonFX(motorId.id(), motorId.bus());
    Request = new MotionMagicVoltage(canCoderOffset);
    coastRequest = new CoastOut();
    canCoder = new CANcoder(canCoderId.id(), canCoderId.bus());

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Voltage.PeakForwardVoltage = 16;
    config.Voltage.PeakReverseVoltage = -16;
    config.CurrentLimits.StatorCurrentLimit = 20;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 10;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = 58.333;
    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .29;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.26;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(config));

    CANcoderConfiguration cc_config = new CANcoderConfiguration();
    cc_config.MagnetSensor.MagnetOffset = canCoderOffset.in(Rotations); // UNIT: ROTATIONS
    // cc_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    // AdvantageScope publishes in radians

    PhoenixUtil.tryUntilOk(5, () -> canCoder.getConfigurator().apply(cc_config));
  }

  // TODO The two setPoints to be used are 0 degrees & 90 degrees (use an enum)
  @Override
  public void setTarget(Angle target) {
    Request = Request.withPosition(target).withSlot(0);
    Motor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void updateInputs(ClawAngleIOInputs inputs) {
    inputs.clawAngle.mut_replace(Motor.getPosition().getValue());
    inputs.clawAngularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.clawAngleSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
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
    slot0Configs.GravityType =
        GravityTypeValue.Elevator_Static; // Not arm because gravity should not take effect
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(motionMagicConfigs));
  }

  @Override
  public void applyCoastMode() {
    Motor.setControl(coastRequest);
  }
}
