package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexerShootingAndIntake {

    ElapsedTime timer = new ElapsedTime();



    Servo indexerServo;
    Servo intakeServoLeft;
    Servo intakeServoRight;
    DcMotorEx shooterMotorTop;
    DcMotorEx shooterMotorBottom;
    DcMotor intakeMotor;
    ColorRangeSensor colorPosA;
    ColorRangeSensor colorPosB;
    ColorRangeSensor colorPosC;



    double SERVO_INTAKE_POS_RIGHT = 0.66;
    double SERVO_INTAKE_POS_LEFT = 0.34;
    double SERVO_TRAVEL_POS_RIGHT = 0.55;
    double SERVO_TRAVEL_POS_LEFT = 0.45;
    double SERVO_TRANSFER_POS_RIGHT = 0.49;
    double SERVO_TRANSFER_POS_LEFT = 0.515;

    double INDEXER_SERVO_POS_A = 0.9;
    double INDEXER_SERVO_POS_B = 0.5;
    double INDEXER_SERVO_POS_C = 0.1;


    public enum ShootingStates {
        SHOOTING_START,
        SHOOTING_A,
        SHOOTING_B,
        SHOOTING_C,
        INDEXER_EMPTY,
        STOP_SHOOTING
    }
    ShootingStates shootingState = ShootingStates.SHOOTING_START;

    boolean areShooterMotorsAtSpeed;



    public IndexerShootingAndIntake(Object[] hardwareList) {
        this.indexerServo = (Servo) hardwareList[0];
        this.intakeServoLeft = (Servo) hardwareList[1];
        this.intakeServoRight = (Servo) hardwareList[2];
        this.shooterMotorTop = (DcMotorEx) hardwareList[3];
        this.shooterMotorBottom = (DcMotorEx) hardwareList[4];
        this.intakeMotor = (DcMotor) hardwareList[5];
        this.colorPosA = (ColorRangeSensor) hardwareList[6];
        this.colorPosB = (ColorRangeSensor) hardwareList[7];
        this.colorPosC = (ColorRangeSensor) hardwareList[8];
    }



    public void shootBalls (double shooterVelocity) {
        switch (shootingState) {
            case SHOOTING_START:
                shootingState = ShootingStates.INDEXER_EMPTY;
                break;
            case STOP_SHOOTING:
                break;
            case INDEXER_EMPTY:
                shooterMotorTop.setVelocity(shooterVelocity);
                shooterMotorBottom.setVelocity(shooterVelocity);
                areShooterMotorsAtSpeed = shooterMotorTop.getVelocity() == -shooterVelocity && shooterMotorBottom.getVelocity() == shooterVelocity;

                intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                intakeMotor.setPower(0);

                if (colorPosA.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_A;
                    break;
                }
                if (colorPosB.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_B;
                    break;
                }
                if (colorPosC.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_C;
                    break;
                }
            case SHOOTING_A:
                intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                intakeMotor.setPower(1);
                while (!areShooterMotorsAtSpeed) {
                    continue;
                }
                timer.reset();
                indexerServo.setPosition(INDEXER_SERVO_POS_A);
                while (timer.milliseconds() < 750) {
                    continue;
                }
                intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);

                if (colorPosB.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_B;
                    break;
                } else if (colorPosC.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_C;
                    break;
                } else {
                    shootingState = ShootingStates.STOP_SHOOTING;
                    break;
                }
            case SHOOTING_B:
                intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                intakeMotor.setPower(1);
                while (!areShooterMotorsAtSpeed) {
                    continue;
                }
                timer.reset();
                indexerServo.setPosition(INDEXER_SERVO_POS_B);
                while (timer.milliseconds() < 750) {
                    continue;
                }
                intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);

                if (colorPosA.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_A;
                    break;
                } else if (colorPosC.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_C;
                    break;
                } else {
                    shootingState = ShootingStates.STOP_SHOOTING;
                    break;
                }
            case SHOOTING_C:
                intakeServoRight.setPosition(SERVO_TRAVEL_POS_RIGHT);
                intakeServoLeft.setPosition(SERVO_TRAVEL_POS_LEFT);
                intakeMotor.setPower(1);
                while (!areShooterMotorsAtSpeed) {
                    continue;
                }
                timer.reset();
                indexerServo.setPosition(INDEXER_SERVO_POS_C);
                while (timer.milliseconds() < 750) {
                    continue;
                }
                intakeServoRight.setPosition(SERVO_TRANSFER_POS_RIGHT);
                intakeServoLeft.setPosition(SERVO_TRANSFER_POS_LEFT);

                if (colorPosA.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_A;
                    break;
                } else if (colorPosB.getDistance(DistanceUnit.MM) < 50) {
                    shootingState = ShootingStates.SHOOTING_B;
                    break;
                } else {
                    shootingState = ShootingStates.STOP_SHOOTING;
                    break;
                }
        }
    }

    public ShootingStates getShooterState () {
        return shootingState;
    }

    public void indexBalls(boolean isTriggerPressed) {

    }





}
