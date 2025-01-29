package frc.robot.subsystems.swervedrive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;


public class Elevator extends ProfiledPIDController{

    Constants.OperatorConstants constants;
    

    final SparkMax m_elevator = new SparkMax(constants.m_elevator, MotorType.kBrushless);
    final SparkMax m_follower = new SparkMax(constants.m_elevatorFollower, MotorType.kBrushless);

    SparkMaxConfig mainConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    final AbsoluteEncoder encoder = m_elevator.getAbsoluteEncoder();

    static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(6, 3);
             
    public Elevator(double p, double i, double d){
        super(p, i, d, constraints);

        mainConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15);

        followerConfig
            .follow(m_elevator)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15);

        m_elevator.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setGoal(getGoal().position);
    }
}
