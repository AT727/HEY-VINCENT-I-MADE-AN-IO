package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  SingleJointedArmSim sim;
  PIDController pid;

  public ArmIOSim() {
    sim = new SingleJointedArmSim(DCMotor.getNEO(1), 0, 0, 0, 0, 0, false, 0);

    pid =
        new PIDController(
            ArmConstants.gains.kP(), ArmConstants.gains.kI(), ArmConstants.gains.kD());
  }

  @Override
  public void setSetpoint(double setpoint, double feedforward) {
    sim.setInputVoltage(pid.calculate(setpoint) + feedforward);
  }

  @Override
  public void setPower(double power) {
    sim.setInputVoltage(power);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.position = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocity = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.output = sim.getOutput(0);
  }
}
