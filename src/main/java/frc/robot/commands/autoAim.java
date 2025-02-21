package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;


public class autoAim extends Command{
    //add motor class used. 
    private PIDController m_PidController;
    private final String limelightName = "limelight";
    private final int targetId = 1;
    public autoAim(){
        m_PidController = new PIDController(0.1, 0, 0);
        m_PidController.setTolerance(2);
        //addRequirements(); - add the subsytem you want to tell the system you are utilizing so two systems cant utilize atht eh same time 
    }

    @Override
    public void initialize(){
        m_PidController.reset();
    }

    @Override
    public void execute(){

       if (LimelightHelpers.getFiducialID(limelightName) == -1){
        //when nothing is detected
       }else{
        //setMotors to tht direction
       }

    }


}
