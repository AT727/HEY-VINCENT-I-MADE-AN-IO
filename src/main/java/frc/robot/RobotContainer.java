// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmIOSim;
import frc.robot.subsystems.ArmIOSpark;

public class RobotContainer {

  Arm arm;

  public RobotContainer() {
    switch(Constants.mode){
      case REAL -> {
        arm = new Arm(new ArmIOSpark());
      }

      case SIM -> {
        arm = new Arm(new ArmIOSim());
      }

      case REPLAY -> {}
    }

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
