package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class LimeLightTrackingAndDistance {

    DcMotor turretMotor;
    LLResult llResult;

    final double SCALEFORDISTANCE = 189.912;
    final double POWERFORDISTANCE = -0.5062153;
    final double SLOPEFORVELOCITY = 1.793985;
    final double BVALUEFORVELOCITY = 900;

    double output;
    double integralSum = 0;
    final double Kp = 0.03; //0.025 initial //0.05to high
    final double Ki = 0.0005;
    final double Kd = 0.0042;

    double lastError = 0;
    double distance = 0;
    double DIRECTION = 0;

    ElapsedTime timer = new ElapsedTime();


    public LimeLightTrackingAndDistance(DcMotor turretMotor) {
        this.turretMotor = turretMotor;
    }

    public void setLlResult(LLResult llResult) {
        this.llResult = llResult;
    }

    public void trackAprilTag () {
        double error = llResult.getTx();
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

//        if (error > 0) {
//            DIRECTION = -1;
//        } else if (error < 0) {
//            DIRECTION = 1;
//        }

        if (error < -1 || error > 1 && llResult.isValid()) {
            turretMotor.setPower(output);
        } else if (!llResult.isValid()) {
            turretMotor.setPower(DIRECTION);
        } else {
            turretMotor.setPower(0);
        }

    }

    public double getOutput() {
        return output;
    }

    public double distanceToTarget() {
        double TA = llResult.getTa();
        distance = SCALEFORDISTANCE * Math.pow(TA, POWERFORDISTANCE);
        return distance;
    }

    public double calculateRPMForShooter() {
        double motorVelocity = (SLOPEFORVELOCITY * distanceToTarget()) + BVALUEFORVELOCITY;
        return Math.ceil(motorVelocity);
    }
}
