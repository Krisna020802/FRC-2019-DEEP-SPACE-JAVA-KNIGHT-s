package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import com.analog.adis16448.frc.ADIS16448_IMU;


import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
public class Robot extends TimedRobot {
  
  private ADIS16448_IMU IMU = new ADIS16448_IMU();
  private PWMVictorSPX BackLifter = new PWMVictorSPX(1);
  private PWMVictorSPX Slider = new PWMVictorSPX(6);
  //private PWMVictorSPX BackSlider = new PWMVictorSPX(0);
  private AnalogInput limitSwitchUP = new AnalogInput(2);
  private AnalogInput limitSwitchDOWN = new AnalogInput(3);
  private CANSparkMax leftFront = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax leftBack = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rightBack = new CANSparkMax(2, MotorType.kBrushless);
  private PWMVictorSPX InTake = new PWMVictorSPX(2);
  private PWMVictorSPX OutTake = new PWMVictorSPX(3);
  private PWMVictorSPX Standar = new PWMVictorSPX(5);
  private CANSparkMax LifterMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANEncoder LifterEncoder = new CANEncoder(LifterMotor);

  //Motor Controller Group
  private DifferentialDrive FrontDrive = new DifferentialDrive(leftFront, rightFront);
  private DifferentialDrive BackDrive = new DifferentialDrive(leftBack, rightBack);

  //Joysticks
  private Joystick joystick = new Joystick(0);
  private Joystick joystick2 = new Joystick(1);
  private Joystick joystick3 = new Joystick(2);

  //Lifter Variables
  public boolean LifterEmergency = true;
  public double lifterSpeed = 0;
  public double destinationpos = 0;
  public double currentpos = 0;
  public double LiftPID = 0.017;
  public double BackSpeed;

  //Limelight Variable
  boolean toggleOn = false;
  boolean togglePressed = false;
  boolean toggleOn1 = false;
  boolean togglePressed1 = false;
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private int LimeMode = 0;
  double WSSpeed;
  double ADSpeed;
  double lastWSSpeed;
  double lastADSpeed;
  double MAX_SPEED_CHANGE_PER_TICK = 0.1;
  double FIXwsspeed;
  double FIXadspeed;

  //Camera Server

  
  
  @Override
  public void robotInit() {

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(320, 240);
      camera2.setResolution(320, 240);
      camera.setFPS(15);
      camera2.setFPS(14);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("blur", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()){
        cvSink.grabFrame(source);
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);

        outputStream.putFrame(output);
      }
    }).start();

    SmartDashboard.getNumber("INI ANGLE YYY = ", IMU.getAngleY());
    SmartDashboard.getNumber("INI ANGLE XXX = ", IMU.getAngleY());
    

  }

  
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    
    
  }
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("encoder", currentpos);
    currentpos = LifterEncoder.getPosition();
    Update_Limelight_Tracking();

    double TSpeed = -(joystick3.getY()*1.5);

    if (TSpeed < 0){
      TSpeed = 0;
    }

   

  //Lifter Position Buttons
  if (joystick.getRawButton(7)){
    destinationpos = 95;
  }
  if (joystick.getRawButton(9)){
    destinationpos = 190;
  }
  if (joystick.getRawButton(11)){
    destinationpos = 316.327;
  }
  if (joystick.getRawButton(8)){ 
    destinationpos = 63.071;
  }
  if (joystick.getRawButton(10)){
    destinationpos = 126.352;
  }
  if (joystick.getRawButton(12)){
    destinationpos = 189.613;
  }


  //BackLifter

  if (joystick3.getRawButton(7) ==  true) {
    BackSpeed = 0.8;
  } else if (joystick3.getRawButton(6) == true) {
    BackSpeed = -1.5;
  }else{
    BackSpeed = 0;
  
  }

  // Slider 

  if (joystick3.getRawButton(8)){
    Slider.set(-1.5);
  } else if (joystick3.getRawButton(9)){
    Slider.set(1.5);
  } else {
    Slider.set(0);
  }


  // Standar

  if (joystick.getRawButton(5)){
    Standar.set(-1.5);
  } else if (joystick.getRawButton(6)){
    Standar.set(1.5);
  } else {
    Standar.set(0);
  }
  
  //Lifter Set Position Activate
    if (joystick.getRawButton(4) == true){
      lifterSpeed = (destinationpos - currentpos) * LiftPID;
      /*
      if (currentpos == destinationpos){
      motorspeed = 0;
    } else if (currentpos < destinationpos){
      motorspeed = 1;
    } else{
      motorspeed = -1;
    }
    */

    //lifter boost or normal mode
      LifterMotor.set(lifterSpeed);
    }else if (joystick.getRawButton(3) == true){
      LifterMotor.set(-joystick.getY());
    }else if (limitSwitchUP.getValue() < 1500 && joystick.getY() > 0) {
        LifterMotor.set(0);
    } else if (limitSwitchDOWN.getValue() < 1500 && joystick.getY() < 0) {
      LifterMotor.set(0);
    } else{
      LifterMotor.set(-joystick.getY()/2);
    }
   
  
  

    //Manual Drive Boost Mode
    if (joystick2.getRawButton(6) == true){
      WSSpeed = (-joystick2.getY() );
      ADSpeed = (joystick2.getRawAxis(4)  );


    //Manual Drive Slow Mode
    } else if(joystick2.getRawButton(1) == true){
      WSSpeed = (-joystick2.getY()/3 );
      ADSpeed = (joystick2.getRawAxis(4)/3  );


    //Limelight AutoDrive
    } else if (joystick3.getRawButton(1) == true) {
      if (m_LimelightHasValidTarget) {
        WSSpeed = m_LimelightDriveCommand;
        ADSpeed = m_LimelightSteerCommand;
      }else{
        WSSpeed = 0.0;
        ADSpeed = 0.0;

    //Manual Drive Normal Mode
    }}else if (joystick3.getRawButton(5) == true) {
      WSSpeed = (-joystick2.getY()/1.5 *TSpeed);
      ADSpeed = (joystick2.getRawAxis(4)/1.5 *TSpeed);
    } else {
      WSSpeed = (-joystick2.getY()/1.5 );
      ADSpeed = (joystick2.getRawAxis(4)/1.5 );
    }



    //Intake And Output
    if (joystick.getRawButton(2) == true)  {
      InTake.set(0.7);
      OutTake.set(0.0);
    } else if (joystick.getRawButton(1) == true){
      OutTake.set(-0.6);
      InTake.set(0.0);
    }else {
      InTake.set(0.0);
      OutTake.set(0.0);
    }

  

    //Motor Set Speed
    FrontDrive.arcadeDrive(WSSpeed, ADSpeed);//(FIXwsspeed, FIXadspeed);
    BackDrive.arcadeDrive(WSSpeed, ADSpeed);//(FIXwsspeed, FIXadspeed);
    BackLifter.set(BackSpeed);

   // WSSpeed = lastWSSpeed;
   // ADSpeed = lastADSpeed;
    
  
  
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("encoder", currentpos);
    currentpos = LifterEncoder.getPosition();
    Update_Limelight_Tracking();

    double TSpeed = -(joystick3.getY() *1.5);
    

    if (TSpeed < 0){
      TSpeed = 0;
    }
   

  //Lifter Position Buttons
  if (joystick.getRawButton(7)){
    destinationpos = 95;
  }
  if (joystick.getRawButton(9)){
    destinationpos = 190;
  }
  if (joystick.getRawButton(11)){
    destinationpos = 316.327;
  }
  if (joystick.getRawButton(8)){ 
    destinationpos = 63.071;
  }
  if (joystick.getRawButton(10)){
    destinationpos = 126.352;
  }
  if (joystick.getRawButton(12)){
    destinationpos = 189.613;
  }


  //BackLifter

  if (joystick3.getRawButton(7) ==  true) {
    BackSpeed = 0.8;
  } else if (joystick3.getRawButton(6) == true) {
    BackSpeed = -1.5;
  }else{
    BackSpeed = 0;
  
  }

  // Slider 

  if (joystick3.getRawButton(8)){
    Slider.set(-1.5);
  } else if (joystick3.getRawButton(9)){
    Slider.set(1.5);
  } else {
    Slider.set(0);
  }

  // Standar

  if (joystick.getRawButton(5)){
    Standar.set(-1.5);
  } else if (joystick.getRawButton(6)){
    Standar.set(1.5);
  } else {
    Standar.set(0);
  }
  
  //Lifter Set Position Activate
    if (joystick.getRawButton(4) == true){
      lifterSpeed = (destinationpos - currentpos) * LiftPID;
      /*
      if (currentpos == destinationpos){
      motorspeed = 0;
    } else if (currentpos < destinationpos){
      motorspeed = 1;
    } else{
      motorspeed = -1;
    }
    */

    //lifter boost or normal mode
      LifterMotor.set(lifterSpeed);
    }else if (joystick.getRawButton(3) == true){
      LifterMotor.set(-joystick.getY());
    }else if (limitSwitchUP.getValue() < 1500 && joystick.getY() > 0) {
        LifterMotor.set(0);
    } else if (limitSwitchDOWN.getValue() < 1500 && joystick.getY() < 0) {
      LifterMotor.set(0);
    } else{
      LifterMotor.set(-joystick.getY()/2);
    }
   
  
  

    //Manual Drive Boost Mode
    if (joystick2.getRawButton(6) == true){
      WSSpeed = (-joystick2.getY() );
      ADSpeed = (joystick2.getRawAxis(4) );


    //Manual Drive Slow Mode
    } else if(joystick2.getRawButton(1) == true){
      WSSpeed = (-joystick2.getY()/3 );
      ADSpeed = (joystick2.getRawAxis(4)/3) ;


    //Limelight AutoDrive
    } else if (joystick3.getRawButton(1) == true) {
      if (m_LimelightHasValidTarget) {
        WSSpeed = m_LimelightDriveCommand;
        ADSpeed = m_LimelightSteerCommand;
      }else{
        WSSpeed = 0.0;
        ADSpeed = 0.0;

    //Manual Drive Normal Mode
    }}else if (joystick3.getRawButton(5) == true){ 
      WSSpeed = (-joystick2.getY()/1.5 *TSpeed);
      ADSpeed = (joystick2.getRawAxis(4)/1.5 *TSpeed);
    }
    else { 
      WSSpeed = (-joystick2.getY()/1.5 );
      ADSpeed = (joystick2.getRawAxis(4)/1.5 );
    }



    //Intake And Output
    if (joystick.getRawButton(2) == true)  {
      InTake.set(0.7);
      OutTake.set(0.0);
    } else if (joystick.getRawButton(1) == true){
      OutTake.set(-0.6);
      InTake.set(0.0);
    }else {
      InTake.set(0.0);
      OutTake.set(0.0);
    }
  

    //Motor Set Speed
    FrontDrive.arcadeDrive(WSSpeed, ADSpeed);//(FIXwsspeed, FIXadspeed);
    BackDrive.arcadeDrive(WSSpeed, ADSpeed);//(FIXwsspeed, FIXadspeed);
    BackLifter.set(BackSpeed);

  // <WSSpeed = lastWSSpeed;
  // ADSpeed = lastADSpeed;
    
  }
    
    public void Update_Limelight_Tracking() {
      NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("pipeline").setNumber(LimeMode);
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.017;                    // how hard to turn toward the target
        final double DRIVE_K = -0.017 ;                    // how hard to drive fwd toward the target
        //final double DESIRED_TARGET_AREA = 40.0;        // Area of the target when the robot reaches the wall
        //final double MAX_DRIVE = 0.01;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("ty").getDouble(0);
        //double ta = NetworkTableInstance.getDefault().getTable("limelight-javakni").getEntry("ta").getDouble(0);

        if (tv < 1.0) {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Limelight AutoDrive Code
        double steer_cmdx = tx * STEER_K;
        double drive_cmdy = ty * DRIVE_K;

        m_LimelightSteerCommand = steer_cmdx;
        m_LimelightDriveCommand = drive_cmdy;

       /* double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
        */
  }

  @Override
  public void testPeriodic() {
  }
}
