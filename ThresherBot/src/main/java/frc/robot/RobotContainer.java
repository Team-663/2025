// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveInputStream;

public class RobotContainer {
   // The robot's subsystems and commands are defined here...
   private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

   // Replace with CommandPS4Controller or CommandJoystick if needed
   private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
   private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
         () -> driverXbox.getLeftY() * -1,
         () -> driverXbox.getLeftX() * -1)
         .withControllerRotationAxis(driverXbox::getRightX)
         .deadband(OperatorConstants.DEADBAND)
         .scaleTranslation(0.8)
         .allianceRelativeControl(true);

   /**
    * Clone's the angular velocity input stream and converts it to a fieldRelative
    * input stream.
    */
   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
         driverXbox::getRightY)
         .headingWhile(true);

   public RobotContainer() {
      // Configure the trigger bindings
      configureBindings();
      DriverStation.silenceJoystickConnectionWarning(true);
      NamedCommands.registerCommand("test", Commands.print("I EXIST"));
   }

   /**
    * Use this method to define your trigger->command mappings. Triggers can be
    * created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
    * an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
    * {@link
    * CommandXboxController
    * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or
    * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */
   private void configureBindings() {

      Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      if (DriverStation.isTest()) {
         drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

         driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
         // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
         driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
         driverXbox.back().whileTrue(drivebase.centerModulesCommand());
         driverXbox.leftBumper().onTrue(Commands.none());
         driverXbox.rightBumper().onTrue(Commands.none());
      } else {
         driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
         driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

         driverXbox.start().whileTrue(Commands.none());
         driverXbox.back().whileTrue(Commands.none());
         driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
         driverXbox.rightBumper().onTrue(Commands.none());
      }
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

   public void setMotorBrake(boolean brake) {
      drivebase.setMotorBrake(brake);
   }
}
