package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {


    public BNO055IMU imu;


    public Limelight3A limelight;


    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor turretMotor;
    public DcMotor testMotor;


    public Servo intakeServoLeft;
    public Servo intakeServoRight;
    public Servo indexerServo;
    //public Servo intakeBall;



    public DcMotorEx shooterMotorOne;
    public DcMotorEx shooterMotorTwo;


    public ColorRangeSensor testColor;

    HardwareMap hwmap;

    public void init(HardwareMap ghwmap) {

        hwmap = ghwmap;

        frontLeftMotor = hwmap.get(DcMotor.class, "fl");
        backLeftMotor = hwmap.get(DcMotor.class, "bl");
        frontRightMotor = hwmap.get(DcMotor.class, "fr");
        backRightMotor = hwmap.get(DcMotor.class, "br");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        turretMotor = hwmap.get(DcMotor.class, "turret");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        testMotor = hwmap.get(DcMotor.class, "test");
        testMotor.setDirection(DcMotor.Direction.FORWARD);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeServoLeft = hwmap.get(Servo.class, "intakeL");
        intakeServoRight = hwmap.get(Servo.class, "intakeR");


        indexerServo = hwmap.get(Servo.class, "indexer");


        //intakeBall = hwmap.get(Servo.class, "ballin");


        shooterMotorOne = hwmap.get(DcMotorEx.class, "sm1");
        shooterMotorTwo = hwmap.get(DcMotorEx.class, "sm2");


        shooterMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        testColor = hwmap.get(ColorRangeSensor.class, "colorOne");


        limelight = hwmap.get(Limelight3A.class, "LimeLight");


    }





}
