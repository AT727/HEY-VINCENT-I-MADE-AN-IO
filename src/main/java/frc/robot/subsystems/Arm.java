package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private ArmInputs inputs;
  private ArmFeedforward feedforward;

  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpoinState = new TrapezoidProfile.State();
  private double goalAngle;

  public Arm(ArmIO io) {
    this.io = io;
    inputs = new ArmInputs();

    feedforward =
        new ArmFeedforward(
            ArmConstants.gains.kS(), ArmConstants.gains.kG(), ArmConstants.gains.kV());
    profile = new TrapezoidProfile(ArmConstants.contraints);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    setpoinState = profile.calculate(0.02, setpoinState, new TrapezoidProfile.State(goalAngle, 0));

    double ff = feedforward.calculate(setpoinState.position, setpoinState.velocity);

    io.setSetpoint(setpoinState.position, ff);
  }

  public void setGoalAngle(double angle) {
    this.goalAngle = angle;
  }
}
