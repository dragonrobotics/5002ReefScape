package frc.robot.commands;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class autoAlign extends Command{

    private PhotonCamera camera;
    private CommandSwerveDrivetrain swerveDrive;
    private SwerveRequest.FieldCentric requester;

    private PIDController rotationController;
    private PIDController xPidController;
    private PIDController yPidController;

    private Boolean complete;
    private int ID;

    
    public autoAlign(CommandSwerveDrivetrain swerveDrive, int ID){
        this.swerveDrive = swerveDrive;
        this.ID = ID;
        addRequirements(swerveDrive);

        camera = new PhotonCamera(OperatorConstants.cameraName);
        requester = new SwerveRequest.FieldCentric();

        rotationController = new PIDController(0.5, 0, 0);
        xPidController = new PIDController(0.5, 0.0, 0.0);
        yPidController = new PIDController(0.5, 0.0, 0.0);
    }

    @Override
    public void initialize() {
        complete = false;
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        if (hasTargets){
            var target = result.getBestTarget();
            if (target.getFiducialId() == this.ID){
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                double distance = OperatorConstants.distanceToTag;  
                

                double rError = bestCameraToTarget.getRotation().getZ();
                double yError = bestCameraToTarget.getY();
                double xError = bestCameraToTarget.getX() - distance;

                double xOutput = xPidController.calculate(xError, 0);
                double yOutput = yPidController.calculate(yError, 0);
                double rOutput = rotationController.calculate(rError, 0);

                swerveDrive.setControl(requester.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(rOutput));
            }
        }
    }

    @Override
    public boolean isFinished(){
        double positionTolerance = 0.02; // 2 cm
        double rotationTolerance = 0.05; // Small angle in radians

        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            if (target.getFiducialId() == this.ID) {
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                double xError = bestCameraToTarget.getX() - (OperatorConstants.distanceToTag);
                double yError = bestCameraToTarget.getY();
                double rError = bestCameraToTarget.getRotation().getZ();

                return Math.abs(xError) < positionTolerance &&
                       Math.abs(yError) < positionTolerance &&
                       Math.abs(rError) < rotationTolerance;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setControl(requester.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
