package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClawIO {
    @AutoLog
    public static class GyroIOInputs {
      public boolean connected = false;
      public Rotation2d yawPosition = new Rotation2d();
      public double yawVelocityRadPerSec = 0.0;
      public double[] odometryYawTimestamps = new double[] {};
      public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }
