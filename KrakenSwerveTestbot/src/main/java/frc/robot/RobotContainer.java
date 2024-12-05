// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
   // The robot's subsystems and commands are defined here...
   private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

   // Replace with CommandPS4Controller or CommandJoystick if needed
   private final CommandXboxController driverXbox = new CommandXboxController(
         OperatorConstants.kDriverControllerPort);

   private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
         "swerve"));

   AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
         () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
               OperatorConstants.LEFT_Y_DEADBAND),
         () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
               OperatorConstants.LEFT_X_DEADBAND),
         () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
               OperatorConstants.RIGHT_X_DEADBAND),
         driverXbox.getHID()::getYButtonPressed,
         driverXbox.getHID()::getAButtonPressed,
         driverXbox.getHID()::getXButtonPressed,
         driverXbox.getHID()::getBButtonPressed);

   Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
         () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
         () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
         () -> driverXbox.getRightX() * -1);

   /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
   public RobotContainer() {
      configureBindings();
   }

   private void configureBindings() {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

   }

   public void setDriveMode() {
      configureBindings();
   }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      return Autos.exampleAuto(m_exampleSubsystem);
   }
}
