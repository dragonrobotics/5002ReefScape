package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.BooleanSupplier;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class Arm extends SubsystemBase {
    

    final SparkMax m_armRotator = new SparkMax(OperatorConstants.m_armRotator, MotorType.kBrushless);
    final SparkMax m_armShooter = new SparkMax(OperatorConstants.m_armShooter, MotorType.kBrushless);

    SparkMaxConfig rotatorConfig = new SparkMaxConfig();
    SparkMaxConfig shooterConfig = new SparkMaxConfig();

    final RelativeEncoder encoder = m_armRotator.getEncoder();
    final AbsoluteEncoder rotatorAbsoluteEncoder = m_armRotator.getAbsoluteEncoder();

    PIDController controller = new PIDController(0.5, 0, 0);

    double factor = 0.0;

    public Arm(){
        rotatorConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true)

        .encoder
            .positionConversionFactor(OperatorConstants.m_armConversionFactor)
            .velocityConversionFactor(OperatorConstants.m_armConversionFactor/60);

        shooterConfig
            .idleMode(IdleMode.kBrake);
        
        
        m_armRotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_armShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller.setTolerance(1);
    }

    public Command moveToPosition(Double position){
        return sequence(
            runOnce(()->{controller.setSetpoint(factor);}),
            run(()->{run();}).until(atGoal())
        );
    }

    public double getMeasurement(){
        return encoder.getPosition();
    }

    public void run(){
        m_armRotator.set(controller.calculate(getMeasurement(), controller.getSetpoint()));
    }
    
    public void runMotor(double d){
        m_armRotator.set(d);
    }

    public void intake(){
        m_armShooter.set(-.80);
    }

    public void outake(){
        m_armShooter.set(0.8);
    }

    public BooleanSupplier atGoal(){
        return () -> controller.atSetpoint();
    }

    public void stop(){
        m_armRotator.set(0.0);
    }

    public void calibrate(){
        factor = getMeasurement();
        factor = 90 / getMeasurement();

        rotatorConfig.alternateEncoder
                .positionConversionFactor(factor)
                .velocityConversionFactor(factor/60);

    }

    public void zero(){
        encoder.setPosition(0);
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm position", getMeasurement());
        SmartDashboard.putNumber("Arm Conversion Factor", factor);
    }
}
