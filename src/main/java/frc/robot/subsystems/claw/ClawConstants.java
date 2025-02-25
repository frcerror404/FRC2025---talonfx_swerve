package frc.robot.subsystems.claw;

public class ClawConstants {
  public static final int clawMotorID = 0;
  public static final int deployMotorID = 0;
  public static final int deployEncoderID = 0;
  public static final double motorReduction = 1.0;
  public static final int currentLimit = 40;
  public static final double voltageControllerVolocity = 10.0;

  public static final ClawGains SimGains = new ClawGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final ClawGains TalonFXGains = new ClawGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    record ClawGains(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
        public ClawGains {
            if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kA < 0 || kG < 0) {
                throw new IllegalArgumentException("Gains must be non-negative");
            }
        }
    }

}
