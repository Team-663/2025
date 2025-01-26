import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CenterOnAprilTag extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;

  //Set these
  private final PIDController xController = new PIDController(0, 0, 0);
  private final PIDController yController = new PIDController(0, 0, 0);
  private final PIDController rotationController = new PIDController(0, 0, 0);

  public CenterOnAprilTag(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveDrive = swerveSubsystem.getSwerveDrive(); // Get the SwerveDrive object from your subsystem
    addRequirements(swerveSubsystem);

    // Set tolerances
    xController.setTolerance(0);
    yController.setTolerance(0);
    rotationController.setTolerance(0);
  }

  @Override
  public void initialize() {
    // Reset the controllers
    xController.reset();
    yController.reset();
    rotationController.reset();
  }

  @Override
  public void execute() {
    // Get the pose of the AprilTag relative to the camera using LimelightHelpers
    LimelightHelpers.PoseEstimate limelightEstimate = 
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-ps"); // Replace "limelight" with your Limelight's name

    if (limelightEstimate.tagCount >= 1) {
      Pose2d tagPose = limelightEstimate.pose;

      // Calculate the error in x, y, and rotation
      double xError = tagPose.getX();
      double yError = tagPose.getY();
      double rotationError = tagPose.getRotation().getRadians();

      // Calculate the PID outputs
      double xOutput = xController.calculate(xError);
      double yOutput = yController.calculate(yError);
      double rotationOutput = rotationController.calculate(rotationError);

      // Calculate module states using YAGSL kinematics
      SwerveModuleState[] moduleStates = 
          swerveDrive.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
              xOutput, 
              yOutput, 
              rotationOutput, 
              swerveSubsystem.getHeading())); 

      // Set the module states
      swerveDrive.setModuleStates(moduleStates, false); //assuming false

      // Display PID values on SmartDashboard for tuning
      SmartDashboard.putNumber("X Error", xError);
      SmartDashboard.putNumber("Y Error", yError);
      SmartDashboard.putNumber("Rotation Error", rotationError);
      SmartDashboard.putNumber("X Output", xOutput);
      SmartDashboard.putNumber("Y Output", yOutput);
      SmartDashboard.putNumber("Rotation Output", rotationOutput);
    }
  }

  @Override
  public boolean isFinished() {
    // Finish when the robot is centered on the AprilTag
    return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot by setting all module states to zero
    swerveDrive.setModuleStates(swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)), false); //assuming false
  }
}
