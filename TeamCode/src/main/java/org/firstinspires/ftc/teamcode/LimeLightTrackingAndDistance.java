package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;


public class LimeLightTrackingAndDistance {

    DcMotor turretMotor;
    LLResult llResult;

    private double SCALEFORDISTANCE = 189.912;
    private double POWERFORDISTANCE = -0.5062153;
    private double SLOPEFORVELOCITY = 1.793985;
    private double BVALUEFORVELOCITY = 900;


    double distance = 0;
    double DIRECTION = 0;


    public LimeLightTrackingAndDistance(DcMotor turretMotor) {
        this.turretMotor = turretMotor;
    }

    public void setLlResult(LLResult llResult) {
        this.llResult = llResult;
    }

    public void trackAprilTag () {
        double TX = llResult.getTx();

        if (TX > 0) {
            DIRECTION = 1;
        } else if (TX < 0) {
            DIRECTION = -1;
        }

        if (TX <= 5 && TX >= -5 && llResult.isValid()) {
            turretMotor.setPower(0);
        } else if (llResult.isValid() == false) {
            turretMotor.setPower(DIRECTION *  0.5);

        } else if(llResult.isValid() == true){
            turretMotor.setPower(TX * 0.025);

        }

    }

    public double distanceToTarget() {
        double TA = llResult.getTa();
        distance = SCALEFORDISTANCE * Math.pow(TA, POWERFORDISTANCE);
        return distance;
    }

    public double calculateRPMForShooter() {
        double motorVelocity = (SLOPEFORVELOCITY * distanceToTarget()) + BVALUEFORVELOCITY;
        return motorVelocity;
    }
}
