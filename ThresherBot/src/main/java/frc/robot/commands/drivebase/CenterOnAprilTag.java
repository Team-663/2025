package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class CenterOnAprilTag extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;


  // PID controllers for X, Y, and heading
  private final PIDController xController = new PIDController(kP, 0, 0);
  private final PIDController yController = new PIDController(kP, 0, 0);
  private final PIDController headingController = new PIDController(kP, 0, 0);

  // Tolerance for each axis
  private static final double kTolerance = 0.1; // Example tolerance, adjust as needed

  // Gains for the PID controllers (tune these!)
  private static final double kP = 0.1; // Proportional gain

  public CenterOnAprilTag(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.getSwerveDrive();
    addRequirements(swerveSubsystem);

    // Set tolerances for the PID controllers
    xController.setTolerance(kTolerance);
    yController.setTolerance(kTolerance);
    headingController.setTolerance(kTolerance);
  }

  @Override
  public void initialize() {
    //
  }

  @Override
  public void execute() {
   
    String limelightName = "limelight-ps";
    boolean isOpenLoop = false;

   // if (LimelightHelpers.getTV(limelightName)) {
      // Get the offsets from the limelight
      double horizontalOffset = LimelightHelpers.getTX(limelightName);
      double verticalOffset = LimelightHelpers.getTY(limelightName);
      Pose2d targetPose = LimelightHelpers.getBotPose2d(limelightName);
      double rotationError = targetPose.getRotation().getDegrees();
      //double headingError = swerveSubsystem.getHeading() 

      // Calculate the chassis speeds using the PID controllers
      double xSpeed = xController.calculate(horizontalOffset);
      double ySpeed = yController.calculate(verticalOffset);
      double omega = headingController.calculate(rotationError);

      // Create a ChassisSpeeds object with the calculated speeds
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omega);

      // Set the module states based on the chassis speeds
      swerveDrive.setModuleStates(swerveDrive.kinematics.toSwerveModuleStates(chassisSpeeds), isOpenLoop);
 //  } else {
      // Stop the robot if no target is detected
    //  swerveDrive.setModuleStates(swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)), isOpenLoop);
   // }
  }

  @Override
  public boolean isFinished() {
    // Check if the robot is within tolerance on all axes
    return xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.setModuleStates(swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0)),false);
  }
}

