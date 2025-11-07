/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;




@TeleOp
public class Teleop extends OpMode {

    RobotHardware robothwde = new RobotHardware();


    double headingOffset;

    RobotDrive robotDrive;
    LimeLightTrackingAndDistance limeLightTrackingAndDistance;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //initializes the robot by passing in its hardware map
        robothwde.init(hardwareMap);

        //This is the constructor for the drive class
        robotDrive = new RobotDrive(robothwde.frontLeftMotor, robothwde.backLeftMotor, robothwde.frontRightMotor, robothwde.backRightMotor);
        limeLightTrackingAndDistance = new LimeLightTrackingAndDistance(robothwde.turretMotor);

        robothwde.limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        //This is the setup for the limelight A3 camera's pipeline
        robothwde.limelight.pipelineSwitch(0);



    }

        /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        robothwde.limelight.start();

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        LLResult llResult = robothwde.limelight.getLatestResult();
        limeLightTrackingAndDistance.setLlResult(robothwde.limelight.getLatestResult());

        double distance = limeLightTrackingAndDistance.distanceToTarget();
        limeLightTrackingAndDistance.trackAprilTag();
        robotDrive.controlerDrive(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x, robothwde.imu, headingOffset);



        telemetry.addData("TX", llResult.getTx());
        telemetry.addData("TA", llResult.getTa());
        telemetry.addData("Valid", llResult.isValid());
        telemetry.addData("Distance", distance);
        telemetry.addData("DIRECTION: ", limeLightTrackingAndDistance.DIRECTION);




        //If statement used to change the orientation for the concentric driving
        if (gamepad1.start) {
            headingOffset = -robothwde.imu.getAngularOrientation().firstAngle;
        }






        if (gamepad1.right_trigger > 0.5) {
            robothwde.shooterMotorOne.setVelocity(-limeLightTrackingAndDistance.calculateRPMForShooter());
            robothwde.shooterMotorTwo.setVelocity(limeLightTrackingAndDistance.calculateRPMForShooter());
        } else {
            robothwde.shooterMotorOne.setVelocity(0);
            robothwde.shooterMotorTwo.setVelocity(0);
        }

        telemetry.addData("velocity", limeLightTrackingAndDistance.calculateRPMForShooter());
        telemetry.addData("PID Motor pow", limeLightTrackingAndDistance.getOutput());





        if (gamepad2.a) {
          robothwde.intakeServoRight.setPosition(0.66); //OUT
          robothwde.intakeServoLeft.setPosition(0.34); //OUT

        } else if (gamepad2.b) {
         robothwde.intakeServoRight.setPosition(0.49); //IN
         robothwde.intakeServoLeft.setPosition(0.515); //IN

        }

        if (gamepad2.right_trigger > 0.5) {
            robothwde.testMotor.setPower(1);
        } else {
            robothwde.testMotor.setPower(0);
        }


        if (gamepad2.y) {
            robothwde.indexerServo.setPosition(1);
        } else if (gamepad2.x) {
            robothwde.indexerServo.setPosition(0.5);
        } else {
            robothwde.indexerServo.setPosition(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
