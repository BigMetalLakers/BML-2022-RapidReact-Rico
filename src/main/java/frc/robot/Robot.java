// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
//    Bracebridge and Muskoka Lakes Secondary School (BMLSS) 2022 Base Robot Program
//    
//    2022 update - This is code is written for the 2022 environment
//
//    This code can be inserted to rewrite the contents of the Robot.java file in src>main>java>frc>robot
//    after using the WPIlib command pallet to create a robot template for a 'timed robot' in java for team 6859
//     
//    This robot program is intended to instruct on the use of both the java programming language
//    and the Worchester Polytechnical Institute code libraries for First Robotics Competition (FRC)
//    or wpilib.  The wpilib is so pervasive in the process, that we cannot run the robot without 
//    using many of the tools it provides.  You can see the wpilib command pallet icon as a small W bordered
//    by a red hexagon in the top right corner.  You use this command pallet to start a new robot project
//    from an example, or from a template.  Many of the examples can be open and run with our 2020 robot,
//    which is configured as the "ASCII art" shows (to be consistent with this document's style guide, I must
//    say just this once that ASCII is an acronym for American Standard Code for Infromation Interchange).
//
//    ┌───┬───────────────────────────────────────────────────┬───┐      -"Spark" is an older pulse-width
//    │   │             top_motor     bottom_motor            │   │       modulation (PWM) motor controller;
//    │ █ │               CAN 22         CAN 21               │ █ │       this robot has four for the drivetrain
//    │ █ │┌──────┐           │           │           ┌──────┐│ █ │      -one Spark can control only one
//    │ █ ││ NEO  ╞═Spark Max─┘   shooter └─Spark Max═╡ NEO  ││ █ │        CIM motor (Chiaphua-Components 
//    │   │└──────┘                                   └──────┘│   │        Industrial Motors)
//    │   │                                                   │   │      -each CIM motor is designed for 12V (volts) 
//    │   │         left                     right            │   │       and produces an average of 0.45 N·m
//    │   │    PWM(1) Signal wire       PWM(0) Signal wire    │   │       (newton-metres) of torque at its 
//    │   │               │                   │               │   │       normal load (27A, 205W output, 4320 rpm)
//    │   │┌──────┐       │                   │       ┌──────┐│   │        (A,amps/ W, watts/ rpm, rotations per minute)
//    │   ││ CIM  ╞═Spark─┤                   ├─Spark═╡ CIM  ││   │      -These motors are powerful.  At the stall
//    │ █ │└──────┘       │                   │       └──────┘│ █ │        torque of 1.2 N·m they can  
//    │ █ │               │                   │               │ █ │        consume 343A for each motor, so
//    │ █ │┌──────┐       │                   │       ┌──────┐│ █ │        don't produce a stituation, such as
//    │   ││ CIM  ╞═Spark─┘                   └─Spark═╡ CIM  ││   │        with an under-geared winch, that stops it
//    │   │└──────┘                                   └──────┘│   │      -The PWM control signal is a ±5V digital  
//    │   │                                                   │   │       signal with a duty cyle (0%-100%)
//    │   │   ┌───────────┐                                   │   │       and polarity that is refered to in 
//    │   │   │16448-IMU  ├── IMU directions                  │   │       wpilib classes as 
//    │   │   └───────────┘ horz. mount ↑Y  →X  · Z(Rico)     │   │       a value from -1.0 to 0.0 to +1.0
//    │ █ │   ┌───────────┐ vert. mount ↑Y  →Z  x X(Goldy)    │ █ │      -The limelight is a rasberry pi computer
//    │ █ │   │ lime-light├── to radio/switch                 │ █ │       with bright green light emmiting diodes
//    │ █ │   └───────────┘                                   │ █ │       (LEDs) and a camera. It is tuned to only
//    │   │                                                   │   │       'see' reflective tape, but when it does,
//    └───┴───────────────────────────────────────────────────┴───┘       it sends the the position of the tape as
//                                                                        variables to the roborio(tx,ty,ta)
//  The inertial measurment unit (IMU) was a necessary enhancement from the sensor built into the roboRio
//  since we have mounted our roboRio in a convenient vertical position.  The onboard gyroscope axis measures
//  angular acceleration perpendicular to the board, which would have been useless for us.
//  With the 2022 Robot, we have a horizontal mount,  so this is no longer necessary.
//  The angle position isn't actually measured by the device; the device measures angular velocity. 
//  The angle is computed with an accumulator that can drift.  If this drift becomes too much of a problem
//  you may need to code your own angle determination from the rate!  Our device also measures some fun but
//  utterly useless things like magnetic field, the temperature of the room and the barometric pressure.
//
//  The symbols below are for ascii art to draw simple diagrams right in the comments.
//  They only show well in editors like notepad or the visual studio environment you are using.
//  They also don't effect the code but give an 'error: unmappable character' when building the code.
//  │┤╡╢╖╕╣║╗╝╜╛┐└┴┬├─┼╞╟╚╔╔╩╦╠═╬╧╨╤╥╙╘╒╓╫╪┘┌█▄▌▐▀   ↑↓→← ±
//
// Dec 29- today I wrote code that takes the raw values from the IMU and finds a rough magnetic heading and 
// initializes a gyroscope heading so they somewhat reference North 
// Jan 10 attempt to re-Write
// Feb 8 - Update to RoboRio2 v4 fix is now available
// Code works on both Goldy and Rico with changes to two lines of code relating to the Yaw Axis
// Feb 25 New code inserted to make CAN bus run SparkMax Motors for Shooting

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//motor controllers
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.RelativeEncoder;                                 //these 4 lines are Feb 25 2022
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Camera
import edu.wpi.first.cameraserver.CameraServer;
// Limelight Support (this is how the imelight camera 
// passes data back and forth between the limelight
// co-processor and the roboRio) 
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// Colour Sensor (Color Sensor v3 from REVROBOTICS)
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.util.Color;
//import com.revrobotics.ColorSensorV3;
// ------ import com.revrobotics.ColorMatchResult;
// ------ import com.revrobotics.ColorMatch;
// ------ import edu.wpi.first.wpilibj.util.Color;

// Motion Sensor
//import edu.wpi.first.wpilibj.AnalogInput;

//IMU -ADIS16488 inertial measurement unit
// import com.analog.adis16448.frc.ADIS16448_IMU; old code
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADIS16448_IMU;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final PowerDistribution m_pdp = new PowerDistribution(); 
  private final Timer m_timer = new Timer();
  
  public static Spark rightSide = new Spark(0); // both right sparks on on one split PWM signal
  public static Spark leftSide = new Spark(1);  // both right sparks on on one split PWM signal
  public static Spark intake = new Spark(2); // intake motor
  public static Spark conveyer = new Spark(3); // intake motor

  public static CANSparkMax top_motor = new CANSparkMax(22,MotorType.kBrushless);  //   Five lines on are Feb 25
  public static CANSparkMax bottom_motor = new CANSparkMax(21,MotorType.kBrushless);
  private SparkMaxPIDController top_pidController;
  private SparkMaxPIDController bottom_pidController;
  private RelativeEncoder top_encoder;
  private RelativeEncoder bottom_encoder;

//motor control variables

  public double setPoint; // shooter speed
  public double kP,kI,kD,kIz,kFF,kMaxOutput,kMinOutput, maxRPM;
  
  public double fastShooter = -8600.0;
  public double slowShooter = -4200.0;

  public double speedSensitivity = 0.7;


  public double intakeSet=0.0;
  public double conveyerSet=0.0;

  XboxController m_driverController = new XboxController(0);
  DifferentialDrive m_robotdrive = new DifferentialDrive(leftSide, rightSide);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  // limelight variables for vision control passed through network tables
  // tx = Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
  // ty = Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
  // ta = Target Area (0% of image to 100% of image)
  // limetarget = boolean value showing if limelight has any valid targets

  public static NetworkTableEntry tx;
  public static NetworkTableEntry ty;
  public static NetworkTableEntry ta;
  public static NetworkTableEntry tv;
  public static double limeX;
  public static double limeY;
  public static double limeArea;
  public static double limeTarget;
  public static boolean limelightTargetAquired;
  public static double limeled;
  public static double limestream;
  
  //This is for the colour sensor
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;  // set to be the correct port for the colour sensor
  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); // this is a colour sensor object
  //------private final ColorMatch m_colorMatcher = new ColorMatch();
  //------private final Color kBlueTarget = Color.kBlue;
  //------private final Color kRedTarget = Color.kRed;

  // Motion Sensor
  //private final AnalogInput ultrasonic = new AnalogInput(0);
  //public static double rawValue; 
  
  // The Inertial Measurement Unit (IMU), ADIS16448 needs the following declaration
  // private ADIS16448_IMU.IMUAxis m_yawActiveAxis;
  // change (kZ for Rico) to (kX for a vertically mounted RoboRIO like Goldy) but remember to change the sign of 
  // gyro Yaw if to do since the Z axis is positive up,and the IMU X axis is positive down!!!!

  //Rover
  //public static final ADIS16448_IMU imu = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s);
  
  //Goldy&Rico
  public static final ADIS16448_IMU imu = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kX, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s);

  
  
  
  public static double gyroYaw;
  //public static double gyroX;
  //public static double gyroY;
  //public static double gyroZ;
  //public static double acclX;
  //public static double acclY;
  //public static double acclZ;
  //public static double magnX;
  //public static double magnY;
  //public static double magnZ;
  //public static double therm;
  //public static double barom;
  //public static double compassHeading; // this is clockwise from the North; positive for E side, negative for W side
  public static double gyroHeading; // this is clockwise from the North; positive for E side, negative for W side
  public final double magneticDeclination = 0.0; // was -10.6; this is a bit useless, but will correct the compass to 'true' north 
  public static double gyroOffset; // this is the initial compass heading, and it is used to correct the gyro which otherwise zeros 
  //                               // in the direction that it is pointed

  //public final double magnX_Offset=0.0; // these are determined by spinning the robot and checking
  //public final double magnY_Offset=0.0;  // the max -min values from the IMU
  //public final double magnZ_Offset=0.0;  // so they correct to the same zero
  public static double targetHeading =0.0;
  public static double goHeading =0.0; // this is the target heading + the POV set by the Xbox
  
  public static double turningFactor;
  public static double speedTrigger;
  public static double angleChange;
  
  


    /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    rightSide.setInverted(true);
    // Feb 25 lines for the shooter are here
   // top_motor.restoreFactoryDefaults();
    //bottom_motor.restoreFactoryDefaults();
  
    top_pidController = top_motor.getPIDController();
    bottom_pidController = bottom_motor.getPIDController();
    
    top_encoder = top_motor.getEncoder();
    bottom_encoder = bottom_motor.getEncoder();
    
    kP = 6.0E-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 1.5E-5;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    top_pidController.setP(kP);
    top_pidController.setI(kI);
    top_pidController.setD(kD);
    top_pidController.setIZone(kIz);
    top_pidController.setFF(kFF);
    top_pidController.setOutputRange(kMinOutput,kMaxOutput);
    

    bottom_pidController.setP(kP);
    bottom_pidController.setI(kI);
    bottom_pidController.setD(kD);
    bottom_pidController.setIZone(kIz);
    bottom_pidController.setFF(kFF);
    bottom_pidController.setOutputRange(kMinOutput,kMaxOutput);
    
    SmartDashboard.putNumber("P Gain" ,kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    

    //m_yawActiveAxis = ADIS16448_IMU.IMUAxis.kX;   // This code is worse than useless-it could have
    //imu.setYawAxis(m_yawActiveAxis);              // been done with arguments in the constructor
    imu.calibrate();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
    
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    
    
    //m_colorMatcher.addColorMatch(kBlueTarget); // colour sensor 
    //m_colorMatcher.addColorMatch(kRedTarget); // 

   // rawValue = ultrasonic.getValue();
   //--- double voltage_scale_factor = 5/RobotController.getVoltage5V();
   //--- double currentDistanceCentimeters = rawValue * voltage_scale_factor * 0.125;
   //--- double currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;

    //magnY = imu.getMagneticFieldY()+magnY_Offset; // offset determined 31 Dec
    //magnZ = imu.getMagneticFieldZ()+magnZ_Offset;
    //gyroOffset = (Math.signum(magnY) !=-1 )? Math.toDegrees(Math.atan(magnZ/magnY)): 180.0 + Math.toDegrees(Math.atan(magnZ/magnY)) ;
    gyroOffset -= magneticDeclination;
    gyroOffset = (Math.signum(gyroOffset) != -1) ? gyroOffset : gyroOffset +360.0;
    gyroOffset = gyroOffset % 360.0;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*
     * Get the current going through channel 7, in Amperes. The PDP returns the
     * current in increments of 0.125A. At low currents
     * the current readings tend to be less accurate.
     */
    SmartDashboard.putNumber("Current Channel 0", m_pdp.getCurrent(0)); 
    SmartDashboard.putNumber("Current Channel 1", m_pdp.getCurrent(1)); 
    SmartDashboard.putNumber("Current Channel 2", m_pdp.getCurrent(2)); 
    SmartDashboard.putNumber("Current Channel 3", m_pdp.getCurrent(3)); 
    SmartDashboard.putNumber("Current Channel 4", m_pdp.getCurrent(4)); 
    SmartDashboard.putNumber("Current Channel 5", m_pdp.getCurrent(5)); 
    SmartDashboard.putNumber("Total Current", m_pdp.getTotalCurrent()); 
    
                                                                      // might be nice to measure them all
    /*
     * Get the voltage going into the PDP, in Volts.
     * The PDP returns the voltage in increments of 0.05 Volts.
     */
    SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());
    /*
     * Retrieves the temperature of the PDP, in degrees Celsius.
     */
    SmartDashboard.putNumber("Temperature", m_pdp.getTemperature());// This is also nice
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
    // read Limelight values periodically
    limeX = tx.getDouble(0.0);
    limeY = ty.getDouble(0.0);
    limeArea = ta.getDouble(0.0);
    limeTarget = tv.getDouble(0.0);
    limelightTargetAquired = (limeTarget==1.0);
    // read the colour sensor
    //Color detectedColor = m_colorSensor.getColor(); 
    /** 
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    String colorString;
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } 
    else if (match.color == kRedTarget) {
      colorString = "Red";
    }
    else {
      colorString = "Unknown";
    }
    */

    // read the IMU values periodically
    gyroYaw = imu.getAngle();
    //gyroX = imu.getGyroAngleX();
    //gyroY = imu.getGyroAngleY();
    //gyroZ = imu.getGyroAngleZ();
    //acclX = imu.getAccelX();
    //acclY = imu.getAccelY();
    //acclZ = imu.getAccelZ();
    //magnX = imu.getMagneticFieldX()+ magnX_Offset;
    //magnY = imu.getMagneticFieldY()+ magnY_Offset;// +30.0; // offset determined 29 Dec
    //magnZ = imu.getMagneticFieldZ()+ magnZ_Offset;// +350.0;
    //barom = imu.getBarometricPressure();
    //therm = imu.getTemperature();
    //double rawValue = ultrasonic.getValue();
    //-double voltage_scale_factor = 5/RobotController.getVoltage5V();
    //-double currentDistanceCentimeters = raw_value * voltage_scale_factor * 0.125;
    //-double currentDistanceInches = raw_value * voltage_scale_factor * 0.0492;


    // Calculate the compassHeading of the robot
    //compassHeading = (Math.signum(magnY) !=-1 )? Math.toDegrees(Math.atan(magnZ/magnY)): 180.0 + Math.toDegrees(Math.atan(magnZ/magnY)) ;
    //compassHeading -= magneticDeclination;
    //compassHeading = (Math.signum(compassHeading) != -1) ? compassHeading : compassHeading +360.0;
    //compassHeading = compassHeading % 360.0;
    // Calculate the gyroHeading from gyroYaw to make it a single positive angle in degressr between zero and 359.999 

    //Rover
    //gyroHeading = gyroYaw;

    //Goldy&Rico
    gyroHeading = -gyroYaw;  // gryoHeading = -gyroYaw;  for Goldy&Rico and gyroHeading = gyroYaw; for Rover   
    
    
    
    gyroHeading += gyroOffset;
    gyroHeading = gyroHeading % 360;
    gyroHeading = (Math.signum(gyroHeading) != -1) ? gyroHeading : gyroHeading +360.0;
   
   
    // post Limelight data to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", limeX);
    SmartDashboard.putNumber("LimelightY", limeY);
    SmartDashboard.putNumber("LimelightArea", limeArea);
    SmartDashboard.putNumber("Limelight Target", limeTarget);
    SmartDashboard.putBoolean("Target Aquired", limelightTargetAquired);
    
    // post the colour sensor values
    //SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    //SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("Proximity", m_colorSensor.getProximity());
    //-SmartDashboard.putNumber("Confidence", match.confidence);
    //-SmartDashboard.putString("Detected Color", colorString);
    //-This next line works, so the matcher is unecessary- the boolean
    //- is true when the ball is blue, which can be made the 'true colour in shuffleboard'
    //SmartDashboard.putBoolean("Color", detectedColor.blue > detectedColor.red);
  
    //SmartDashboard.putNumber("Distance ",rawValue );


    // ... and the IMU too !
    SmartDashboard.putNumber("GyroYaw", gyroYaw);
    //SmartDashboard.putNumber("IMUGyroX", gyroX);
    //SmartDashboard.putNumber("IMUGyroY", gyroY);
    //SmartDashboard.putNumber("IMUGyroZ", gyroZ);
    //SmartDashboard.putNumber("IMUAcclX", acclX);
    //SmartDashboard.putNumber("IMUAcclY", acclY);
    //SmartDashboard.putNumber("IMUAcclZ", acclZ);
    //SmartDashboard.putNumber("IMUMagnX", magnX);
    //SmartDashboard.putNumber("IMUMagnY", magnY);
    //SmartDashboard.putNumber("IMUMagnZ", magnZ);
    //SmartDashboard.putNumber("IMURawPresssure", barom);
    //SmartDashboard.putNumber("IMUTherm", therm);
    
    SmartDashboard.putNumber("Gyro Offset", gyroOffset);
    //SmartDashboard.putNumber("Compass Heading", compassHeading);
    SmartDashboard.putNumber("Gyro Heading", gyroHeading);
    SmartDashboard.putNumber("Target Heading", targetHeading);
    SmartDashboard.putNumber("Turning Factor", turningFactor);
    SmartDashboard.putNumber("Go Heading", goHeading);
   //
   SmartDashboard.putNumber("SetPoint",setPoint);
   SmartDashboard.putNumber("Top Motor Speed",top_encoder.getVelocity());
   SmartDashboard.putNumber("Bottom Motor Speed",bottom_encoder.getVelocity());
  
  
   //SmartDashboard.putNumber("Fast Shooter Speed", fastShooter);
   //SmartDashboard.putNumber("Slow Shooter Speed", slowShooter);
   
   double fast = SmartDashboard.getNumber("Fast Shooter Speed", fastShooter);
   double slow = SmartDashboard.getNumber("Slow Shooter Speed", slowShooter);
   
    if ((fast != fastShooter)) {fastShooter = fast;}
    if ((slow != slowShooter)) {slowShooter = slow;}

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();
    targetHeading = gyroHeading;
    goHeading = targetHeading;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        // This code only works with a functioning IMU
        
        conveyer.set(0);
        angleChange = ((goHeading-gyroHeading ) > -180.0) ? goHeading - gyroHeading: 360.0 + goHeading - gyroHeading;
        angleChange = (angleChange > 180.0) ? angleChange - 360.0: angleChange; 
        turningFactor = angleChange*0.002 + 0.48*Math.signum(angleChange);
        if (m_timer.get() < 3.5) {                   // back off the tarmac and spool up the shoot
          setPoint = fastShooter;
          intake.set(-0.7);
          top_pidController.setReference(setPoint*0.6,CANSparkMax.ControlType.kVelocity);
          bottom_pidController.setReference(-setPoint,CANSparkMax.ControlType.kVelocity);
          goHeading = targetHeading;
          m_robotdrive.arcadeDrive(0.6, 0); // drive forwards half speed away ( 3 seconds at 0.6 WITH turing is about 1.1 m )
        }
        else if(m_timer.get() < 6.5) {              // Line up the with the limelight
          turningFactor = limeX*.07 + .32 * Math.signum(limeX);
          speedTrigger = (limeY*.03 + 0.4 * Math.signum(limeY));
          m_robotdrive.arcadeDrive(speedTrigger, turningFactor);
          //goHeading = (targetHeading + 180 ) % 360.0;
          //m_robotdrive.arcadeDrive(0.0, turningFactor );
          
        }
        else if(m_timer.get() < 7.9) {              // Make the shot
         // goHeading = (targetHeading) % 360.0;
         // m_robotdrive.arcadeDrive(-0.6, 0.0);
          conveyer.set(-0.6); 
        }
        else if(m_timer.get() < 8.4 ) {              // spool back
          //top_pidController.setReference(0,CANSparkMax.ControlType.kVelocity);
          //bottom_pidController.setReference(0,CANSparkMax.ControlType.kVelocity);
          //goHeading = (targetHeading + 180 ) % 360.0;
          //m_robotdrive.arcadeDrive(0.6, turningFactor );
        }
        else if(m_timer.get() < 8.8) {              // go forwards to align
          //conveyer.set(+0.6);
          //goHeading = (targetHeading) % 360.0;
          //m_robotdrive.arcadeDrive(0.0,turningFactor );
        }
        else if(m_timer.get() < 12.9) {   
          //conveyer.set(-0.6);                      // make the second shot
          top_pidController.setReference(setPoint*0.6,CANSparkMax.ControlType.kVelocity);
          bottom_pidController.setReference(-setPoint,CANSparkMax.ControlType.kVelocity);
          //goHeading = (targetHeading + 180 ) % 360.0;
          //goHeading = (targetHeading) % 360.0;
          //m_robotdrive.arcadeDrive(0.6,turningFactor );
        }
        else if(m_timer.get() < 14.9) {             // turn around and wait for the bell- that's all the autonomous time you get!
          //goHeading = (targetHeading-180) % 360.0;
          //m_robotdrive.arcadeDrive(0,turningFactor );
        }
        else {
          m_robotdrive.stopMotor(); // stop robot
          conveyer.set(0);
          intake.set(0);
          top_pidController.setReference(0,CANSparkMax.ControlType.kVelocity);
          bottom_pidController.setReference(0,CANSparkMax.ControlType.kVelocity);
        }
        
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double p = SmartDashboard.getNumber("P Gain",kP);
    double i = SmartDashboard.getNumber("I Gain",kI);
    double d = SmartDashboard.getNumber("D Gain",kD);
    double iz = SmartDashboard.getNumber("I Zone",kIz);
    double ff = SmartDashboard.getNumber("Feed Forward",kFF);
    double max = SmartDashboard.getNumber("Max Output",kMaxOutput);
    double min = SmartDashboard.getNumber("Min Output",kMinOutput);    

    if((p!= kP)) {top_pidController.setP(p);bottom_pidController.setP(p); kP=p;}
    if((i!= kI)) {top_pidController.setI(i);bottom_pidController.setI(i); kI=i;}
    if((d!= kD)) {top_pidController.setD(d);bottom_pidController.setD(d); kD=d;}
    if((iz!= kIz)) {top_pidController.setIZone(iz);bottom_pidController.setIZone(iz); kIz=iz;}
    if((ff!= kFF)) {top_pidController.setFF(ff);bottom_pidController.setFF(ff); kFF=ff;}
    if((max != kMaxOutput) || (min != kMinOutput)) {
      top_pidController.setOutputRange(min, max);
      bottom_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }    
    //setPoint = m_driverController.getRightY()*maxRPM;
    // Set the shooter to intake at 60 rpm unless the B button is pressed
    // When the B button is down, the robot will speed up it's drive to 0.999 speed
    speedSensitivity = m_driverController.getRawButton(2) ? 0.999 : 0.7;//A button fixes distance towards to vision target!
    // intake motor on button top buttons (5&6) 
    setPoint = 1000; 
    setPoint = m_driverController.getRawButton(5) ? slowShooter : setPoint;
    setPoint = m_driverController.getRawButton(6) ? fastShooter : setPoint;

    top_pidController.setReference(setPoint*0.6,CANSparkMax.ControlType.kVelocity);
    bottom_pidController.setReference(-setPoint,CANSparkMax.ControlType.kVelocity);
 
    

    turningFactor = m_driverController.getRawAxis(0) * 0.68; // left stick control of the turning is done first
    speedTrigger = m_driverController.getRawAxis(3) * speedSensitivity  - m_driverController.getRawAxis(2) * speedSensitivity;
    
    // set target heading to the current gyroscope heading with X-box controller Y button 
    targetHeading = m_driverController.getRawButton(4) ? gyroHeading : targetHeading;

    // set the go heading to be modified by the POV pad (arrow pad) -'up' turns the robot to the current target heading
    goHeading = (m_driverController.getPOV() != -1) ?(m_driverController.getPOV() + targetHeading) % 360.0 : targetHeading; 
    //this next 2 expressions set the angleChange to be the smallest angle in the correct direction so the robot doesn't "unwind"
    angleChange = ((goHeading-gyroHeading ) > -180.0) ? goHeading - gyroHeading: 360.0 + goHeading - gyroHeading;
    angleChange = (angleChange > 180.0) ? angleChange - 360.0: angleChange; 
  
    //this sets the turning factor to a proportional control plus a constant - some tuning may improve the performance of this expression
    //turn with POV button - it only works when the POV pad is pressed  
    turningFactor = (m_driverController.getPOV() != -1) ? angleChange*0.002 + 0.48*Math.signum(angleChange):turningFactor;    

    turningFactor = m_driverController.getRawButton(3) ? limeX*.07 + .32 * Math.signum(limeX):turningFactor;// X button turns towards to vision target!
    speedTrigger = m_driverController.getRawButton(1) ? (limeY*.03 + 0.4 * Math.signum(limeY)):speedTrigger;//A button fixes distance towards to vision target!

   

    //intakeSet = m_driverController.getRawButton(5) ? 0.6 : 0.0;// Sets the motor speed for the intake
    //intakeSet = m_driverController.getRawButton(6) ? -0.6 : intakeSet;// Sets the motor speed for the intake
    intakeSet = m_driverController.getRightY()*0.8;
    
    conveyerSet = intakeSet*0.8;
    

    conveyer.set(conveyerSet);
    intake.set(intakeSet);

    m_robotdrive.arcadeDrive(speedTrigger, turningFactor);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}