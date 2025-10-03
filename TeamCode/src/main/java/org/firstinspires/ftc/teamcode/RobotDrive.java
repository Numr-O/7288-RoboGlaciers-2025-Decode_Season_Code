package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotDrive {

    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public double botHeading;


    public RobotDrive (DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;

    }




    public void controlerDrive(double Y, double X, double RX, BNO055IMU imu, double headingOffset) {


        botHeading = -imu.getAngularOrientation().firstAngle - headingOffset;

        double rotX = X * Math.cos(botHeading) - Y * Math.sin(botHeading);
        double rotY = X * Math.sin(botHeading) + Y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(Y) + Math.abs(X) + Math.abs(RX), 1);

        //Front Left Wheel Power
        double frontLeftP = (rotY + rotX + RX) / denominator;
        //Back Left Wheel Power
        double backLeftP = (rotY - rotX + RX) / denominator;
        //Front Right Wheel Power
        double frontRightP = (rotY - rotX - RX) / denominator;
        //Back Right Wheel Power
        double backRightP = (rotY + rotX - RX) / denominator;

        frontLeft.setPower(frontLeftP);
        backLeft.setPower(backLeftP);
        frontRight.setPower(frontRightP);
        backRight.setPower(backRightP);


    }



}