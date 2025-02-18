package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.EncoderConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    Constants.OperatorConstants constants;

    final SparkMax m_armRotator = new SparkMax(constants.m_armRotator, MotorType.kBrushless);
    final SparkMax m_armShooter = new SparkMax(constants.m_armShooter, MotorType.kBrushless);

    SparkMaxConfig rotatorConfig = new SparkMaxConfig();
    SparkMaxConfig shooterConfig = new SparkMaxConfig();

    EncoderConfig rotatorEncoderRelativeConfig = new EncoderConfig();
    EncoderConfig rotatorEncoderAbsoluteConfig = new EncoderConfig();

    final RelativeEncoder rotatorRelativeEncoder = m_armRotator.getEncoder();
    final AbsoluteEncoder rotatorAbsoluteEncoder = m_armRotator.getAbsoluteEncoder();
    
    final DigitalInput beamBreak = new DigitalInput(constants.beamBreakId);

    static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(6, 3);
    ProfiledPIDController controller = new ProfiledPIDController(0.5, 0, 0, constraints);

    public Arm(double p, double i, double d){
        rotatorConfig
            .idleMode(IdleMode.kBrake);
        
        rotatorConfig.encoder
            .positionConversionFactor(1080);
            
        shooterConfig
            .idleMode(IdleMode.kBrake);
        
        m_armRotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_armShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        controller.setGoal(0);
    }

    public Command moveToPosition(Double position){
        return sequence(
            runOnce(()->{controller.setGoal(position);}),
            run(()->{run();}).until(controller::atGoal)
        );
    }

    public double getMeasurement(){
        double remainder = rotatorAbsoluteEncoder.getPosition() / 3.0;
        return remainder;
    }

    public void run(){
        m_armRotator.set(controller.calculate(getMeasurement(), controller.getGoal()));
    }
    
    public void intake(){
        m_armShooter.set(-.80);
    }

    public void outake(){
        m_armShooter.set(0.8);
    }

    public boolean hasCoral(){
        return !beamBreak.get(); 
    }

    public void stop(){
        m_armRotator.stopMotor();
        m_armShooter.stopMotor();
    }
}
