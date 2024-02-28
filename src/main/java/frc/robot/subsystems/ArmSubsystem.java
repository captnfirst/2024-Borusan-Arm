// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public CANSparkMax AngleMotorLeader = new CANSparkMax(10, MotorType.kBrushless); // motor tanımlaması
  public CANSparkMax AngleMotorFollower = new CANSparkMax(11, MotorType.kBrushless); // motor tanımlaması
  RelativeEncoder m_angleEncoder; // sparkmax encoder tanımlaması
  SparkPIDController m_anglePidController; // sparkmax kendi içinde pid yapıcısı tanımlaması
  double kP, kMaxOutput, kMinOutput; // pid değerleri için gerekli olan değişkenler

  public ArmSubsystem() {
    AngleMotorLeader.restoreFactoryDefaults(); // motoru fabrika ayarlarına sıfırlayıp kendi ayarlarımı girmek için kullanıyorum
    AngleMotorLeader.setSmartCurrentLimit(40); // motoru ve sürücüyü yakmamak için akım koruma ekliyorum
    AngleMotorLeader.setIdleMode(IdleMode.kBrake); // motorun istediğim değerde tutması için motoru brake moda alıyorum

    AngleMotorFollower.restoreFactoryDefaults(); // motoru fabrika ayarlarına sıfırlayıp kendi ayarlarımı girmek için kullanıyorum
    AngleMotorFollower.setSmartCurrentLimit(40); // motoru ve sürücüyü yakmamak için akım koruma ekliyorum
    AngleMotorFollower.setIdleMode(IdleMode.kBrake); // motorun istediğim değerde tutması için motoru brake moda alıyorum

    AngleMotorFollower.follow(AngleMotorLeader); // ikinci motorumun birinci motorumla aynı şekilde çalışmasını istediğim için onu takip etmesini istedim

    m_angleEncoder = AngleMotorLeader.getEncoder(); // sparkmax encoderini motorun encoderine eşitliyorum 
    m_anglePidController = AngleMotorLeader.getPIDController(); // sparkmax pid sini motorun pid sine eşitliyorum

    // PID coefficients
    kP = 0.00005; // burada sadece p değeri kullanarak PID nin daha kararlı çalışmasını sağlamaya çalışıyorum.
    kMaxOutput = 1; // PID nin motora verebileceği max hız değeri
    kMinOutput = -1; // PID nin motora verebileceği min hız değeri

    // set PID coefficients
    m_anglePidController.setP(kP); // PID nin p değerini Hesaplama için sparkmax pid gönderdim
    m_anglePidController.setOutputRange(kMinOutput, kMaxOutput); // aynı şekilde burada da bilgileri gönderdim

    m_anglePidController.setSmartMotionMaxVelocity(7500, 0); 
    m_anglePidController.setSmartMotionMinOutputVelocity(0, 0);
    m_anglePidController.setSmartMotionMaxAccel(5000, 0);
    m_anglePidController.setSmartMotionAllowedClosedLoopError(1, 0); 
    // PID nin + özelliği olan hız ve ivmelenmeyi kullanarak istediğim hız aralığında double değerlerin atıyorum 0.65 gelmesi yerine 0.654674564 gibi değerlerle istediğimiz konumda tutmasını sağlıyor
  }

  public void setIntakeAngle(double angle) {
    double targetPosition = (angle / 360.0) * 42 / (1/60); // gönderdiğimiz açı değerini pozizyona çevirerek motora o açıda durmasını söylüyoruz
    m_anglePidController.setReference(MathUtil.clamp(targetPosition, 0, 500), ControlType.kSmartMotion); 
    // mathutil.clamp fonksiyonu ise bize alt ve üst sınırlı çalışmamızı sağlıyor böylece sistem bu verdiğimiz değerlerden daha fazla dışarı çıkamıyor
  }

  public double getIntakePosition() {
    return AngleMotorLeader.getAbsoluteEncoder().getPosition(); // burada da motordan gelen pozizyon değerini çektiriyoruz
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Position", getIntakePosition());
    //Smart Dashboarda gelen değeri yazdırmak için kullanıyoruz
  }
}
