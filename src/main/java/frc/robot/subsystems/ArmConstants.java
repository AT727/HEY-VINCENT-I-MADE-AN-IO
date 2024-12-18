package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class ArmConstants {
  public static final int kPort = 0;
  public static final boolean kInverted = false;
  public static final double kNominalVolts = 12.0;
  public static final int kCurrentLimit = 60;
  public static final IdleMode kIdleMode = IdleMode.kBrake;

  public static final double kPositionConversionFactor = 1;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

  public static final TrapezoidProfile.Constraints contraints =
      new TrapezoidProfile.Constraints(1, 1);

  public static final Gains gains =
      switch (Constants.mode) {
        case REAL -> new Gains(0, 0, 0, 0, 0, 0);
        case SIM -> new Gains(0, 0, 0, 0, 0, 0);
        case REPLAY -> new Gains(0, 0, 0, 0, 0, 0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kG, double kV) {}
}
