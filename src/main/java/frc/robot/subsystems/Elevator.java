package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;


public class Elevator extends SubsystemBase{

    //goes from 0 to 28

    Constants.OperatorConstants constants;

    final SparkMax m_elevator = new SparkMax(OperatorConstants.m_elevator, MotorType.kBrushless);
    final SparkMax m_follower = new SparkMax(OperatorConstants.m_elevatorFollower, MotorType.kBrushless);

    SparkMaxConfig mainConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    final RelativeEncoder encoder = m_elevator.getEncoder();

    static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(6, 3);
    
    ProfiledPIDController controller = new ProfiledPIDController(0.5, 0, 0, constraints);
    
    public Elevator(){
        
        SmartDashboard.putData("Elevator PID", controller);

        mainConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15);

        followerConfig
            .follow(m_elevator)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15);

        m_elevator.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller.setGoal(0.0);
        controller.enableContinuousInput(0, 1);
    }

    //moves the elevator to a position in inches
    public Command moveToPosition(Double position){
        return sequence(
            runOnce(()->{controller.setGoal(position);}),
            run(()->{runElevator();}).until(controller::atGoal)
        );
    }

    public void runElevator(){
        m_elevator.set(controller.calculate(getMeasurement(), controller.getGoal()));
    }

    public void setMotor(Double speed){
        m_elevator.set(speed);
    }

    public void stopMotor(){
        m_elevator.stopMotor();
    }

    public void zeroEncoder(){
        encoder.setPosition(0);
    }
    
    public double getMeasurement(){
        return encoder.getPosition();
    }

    public double getAmps(){
        return m_elevator.getOutputCurrent();
    }

    public void calibrate(){
        Double factor = getMeasurement();
        factor = 17.5 / getMeasurement();


        mainConfig.alternateEncoder
                .positionConversionFactor(factor)
                .velocityConversionFactor(factor/60);

        followerConfig.alternateEncoder
                .positionConversionFactor(factor)
                .velocityConversionFactor(factor/60);
                
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    }


}
