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

package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

// Based on HardwarePushbot
public class Hardware6417
{
    /* Public OpMode members. */
    public DcMotorEx frontLeft = null, frontRight = null, backLeft = null, backRight = null, Arm = null, Carousel = null;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    public Servo Grab = null;

    public ColorSensor color;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    private double RADIUS = 0;
    private double CPR = 753.2;
    private double CIRC = 13.25;
    private double CALIBRATION = 1.1;

    /* Constructor */
    public Hardware6417(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motor
        frontLeft = hwMap.get(DcMotorEx.class, "FrontLeft");
        backLeft = hwMap.get(DcMotorEx.class, "BackLeft");
        frontRight = hwMap.get(DcMotorEx.class, "FrontRight");
        backRight = hwMap.get(DcMotorEx.class, "BackRight");
        Arm = hwMap.get(DcMotorEx.class, "Arm");
        Carousel = hwMap.get(DcMotorEx.class, "Carousel");
        Grab = hwMap.get(Servo.class, "Grab");

        // Set motor and servo directions based on orientation of motors on robot
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        Carousel.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        Arm.setPower(0);
        Carousel.setPower(0);

        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


    }




    public void drivetoPosition(int d, double power){

        power *= 0.75;

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distance = (int) (CALIBRATION * (CPR * d) / CIRC);

        //int distance = (int) CPR;

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setTargetPosition(distance);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while(Math.abs(backLeft.getCurrentPosition()) < backLeft.getTargetPosition()){

        } // || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()

        stop();

        //frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void driveAndStop(int d, double power, LinearOpMode instance){

        power *= 0.85;

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distance = (int) (CALIBRATION * (CPR * d) / CIRC);

        //int distance = (int) CPR;

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setTargetPosition(distance);

        frontLeft.setPower(power);
        frontRight.setPower(power * 0.70);
        backLeft.setPower(power);
        backRight.setPower(power * 0.70);

        while(Math.abs(frontRight.getCurrentPosition()) < Math.abs(frontRight.getTargetPosition())){

        } // || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()

        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        instance.sleep(100);

        stop();

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void strafeToPosition(int d, double power, LinearOpMode instance){

        power *= 0.75;

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distance = (int) ((CPR * d) / CIRC);

        //int distance = (int) CPR;

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setTargetPosition(distance);

        frontLeft.setPower(-power * 0.55);
        frontRight.setPower(power * (1/0.75));
        backLeft.setPower(power * 0.75);
        backRight.setPower(-power * 0.45);

        while(Math.abs(backLeft.getCurrentPosition()) < backLeft.getTargetPosition()){

        } // || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);

        instance.sleep(100);

        stop();

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setDriveSpeeds(double leftVert, double rightVert, double leftHoriz, double rightHoriz) {

        double frontLeftSpeed = leftVert - leftHoriz; //-correction
        double frontRightSpeed = rightVert + rightHoriz; //+correction
        double backLeftSpeed = leftVert + leftHoriz; //-correction
        double backRightSpeed = rightVert - rightHoriz; //+correction

        double largest = 1.0;
        largest = Math.max(largest, Math.abs(frontLeftSpeed));
        largest = Math.max(largest, Math.abs(frontRightSpeed));
        largest = Math.max(largest, Math.abs(backLeftSpeed));
        largest = Math.max(largest, Math.abs(backRightSpeed));

        frontLeft.setPower(frontLeftSpeed / largest);
        frontRight.setPower(frontRightSpeed / largest);
        backLeft.setPower((backLeftSpeed / largest)); //* (1.0/0.6)
        backRight.setPower(backRightSpeed / largest);


    }

    public void grab(double pos){
        Grab.setPosition(pos);
    }

    public void stop() {

        // Set all motors to 0 power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void rotate(int degrees, double power, LinearOpMode instance)
    {
        double  leftPower, rightPower;

        degrees *= -0.8;

        power = Math.min(0.7, power);

        //degrees *= -1;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (instance.opModeIsActive() && getAngle() == 0) {}

            while (instance.opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (instance.opModeIsActive() && getAngle() < degrees) {}

        frontLeft.setPower(-leftPower);
        frontRight.setPower(-rightPower);
        backLeft.setPower(-leftPower);
        backRight.setPower(-rightPower);

        instance.sleep(50);

        stop();

        // wait for rotation to stop.
        //instance.sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

}