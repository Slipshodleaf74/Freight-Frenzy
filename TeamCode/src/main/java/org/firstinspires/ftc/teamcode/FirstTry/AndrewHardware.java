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
 * Motor channel:  Front Left  drive motor:        "FrontLeft"
 * Motor channel:  Front Right drive motor:        "FrontRight"
 * Motor channel:  Back Left drive motor:          "BackLeft"
 * Motor channel:  Back Right drive motor:         "BackRight"
 * Motor channel:  Arm motor:                      "Arm"
 * Motor channel:  Carousel spinner motor:         "Carousel"
 * Servo channel:  Grabber servo:                  "Grab"
 */

// Based on HardwarePushbot
public class AndrewHardware {

    /* public op mode members */
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;
    public DcMotorEx arm = null;
    public DcMotorEx carousel = null;

    public Servo grab1 = null;
    public Servo grab2 = null;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* change these if needed
    private double RADIUS = 0;
    private double CPR = 753.2;
    private double CIRC = 13.25;
    private double CALIBRATION = 1.1;
    */

    /* Constructor */
    public AndrewHardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap){
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motor and servo
        frontLeft = hwMap.get(DcMotorEx.class, "FrontLeft");
        backLeft = hwMap.get(DcMotorEx.class, "BackLeft");
        frontRight = hwMap.get(DcMotorEx.class, "FrontRight");
        backRight = hwMap.get(DcMotorEx.class, "BackRight");
        arm = hwMap.get(DcMotorEx.class, "Arm");
        carousel = hwMap.get(DcMotorEx.class, "Carousel");
        grab1 = hwMap.get(Servo.class, "Grab1");


        // Set motor and servo directions based on orientation of motors on robot
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        /*
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        */

        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        arm.setPower(0);
        carousel.setPower(0);

        // Run motors without encoders
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    /* functions */
    /* functions needed:
        rotate to pos
        strafe to pos
        move to pos
        move * could include rotating and strafing
        get angle
        reset angle? *used in hardware 6417 don't know purpose might need
     */
    public void moveArm(double power){
        arm.setPower(power);
    }

    public void grab(double grabPos){
        double grabPos1 = 0;
        double grabPos2 = 0;
        if (grabPos < 0.5) {
            //open
            grabPos1 = 0.8;
            grabPos2 = 0.1;
        }
        else {
            //grabing
            grabPos1 = 0.55;
            grabPos2 = 0.3;
        }

        grab1.setPosition(grabPos1);
        grab2.setPosition(grabPos2);
    }

    public void spin(double power, boolean direction){
        if(direction){
            carousel.setPower(power);
        }
        else{
            carousel.setPower(-power);
        }
    }

    public void setDriveSpeeds(double forward, double strafe, double rotate, double correction) {

        double frontLeftSpeed = forward + strafe + rotate - correction; //-correction
        double frontRightSpeed = forward - strafe - rotate + correction; //+correction
        double backLeftSpeed = forward - strafe + rotate - correction; //-correction
        double backRightSpeed = forward + strafe - rotate + correction; //+correction

        double largest = 1.0;
        largest = Math.max(largest, Math.abs(frontLeftSpeed));
        largest = Math.max(largest, Math.abs(frontRightSpeed));
        largest = Math.max(largest, Math.abs(backLeftSpeed));
        largest = Math.max(largest, Math.abs(backRightSpeed));

        frontLeft.setPower(frontLeftSpeed / largest);
        frontRight.setPower(frontRightSpeed / largest);
        backLeft.setPower(backLeftSpeed / largest);
        backRight.setPower(backRightSpeed / largest);

        /*
        if(rotate!=0){
            resetAngle();
        }
        */
    }

    public void stop() {
        // Set all motors to 0 power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
