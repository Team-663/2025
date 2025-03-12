// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drivebase.DriveStraightUntilAtDistCmd;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoStraightToSelectedLevel extends SequentialCommandGroup {
  Arm m_arm;
  SwerveSubsystem m_swerve;
  /** Creates a new autoStraightToSelectedLevel. */
  public autoStraightToSelectedLevel(SwerveSubsystem swerve, Arm arm) {
    m_arm = arm;
    m_swerve = swerve;
    addRequirements(arm);
    addRequirements(swerve);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        m_arm.moveArmToNeutralCmd()
         ,m_arm.armPrepCoralCmd()
         ,new DriveStraightUntilAtDistCmd(m_swerve, Constants.AUTO_LASER_DIST_AT_BUMPERS, false).withTimeout(3.0)
         ,new WaitCommand(0.15)
         ,m_arm.armScoreCoralCmd().withTimeout(3.0)


    );
  }
}
