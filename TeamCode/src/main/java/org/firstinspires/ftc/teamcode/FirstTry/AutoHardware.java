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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
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
public class AutoHardware {

    /* public op mode members */
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;
    public DcMotorEx arm = null;
    public DcMotorEx carousel = null;
    public DcMotorEx extender = null;
    public OpenCvCamera Webcam;
    public DcMotorEx intake = null;

    public Servo grab1 = null;
    public Servo wrist = null;
    public Servo flipper = null;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    double speed = 0.75;

    // calculate these in inches
    private double DIAMETER = 3.77952756;
    private double CPR = 800;
    private double CIRC = DIAMETER * Math.PI;
    private double CALIBRATION = 1.0;

    /* calculate these using measurement
    arm and extender encoder values, default values for red side carousel
    private int highPos = 2000;
    private int midPos = 2200;
    private int lowPos = 2400;
    private int vertPos = 1250;

    private int extendMin = -50;
    private int extendHigh = -800;
    private int extendMid = -600;
    private int extendLow = -400;
     */

    //values for arm and extender maintenance and wrist safety
    private double armZeroAnglePosition = 600;
    private double armClicksPerHalfRevolution = 1950;

    private double wristZeroAnglePosition = 0.52;
    private double wristClicksPerHalfRevolution = 0.85 - 0.18;

    private double safeWrist = .7;

    /* Constructors */
    public AutoHardware(){
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
        extender = hwMap.get(DcMotorEx.class, "Extender");
        intake = hwMap.get(DcMotorEx.class, "Intake");

        grab1 = hwMap.get(Servo.class, "Grab1");
        wrist = hwMap.get(Servo.class, "Wrist");
        flipper = hwMap.get(Servo.class, "Flipper");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor and servo directions based on orientation of motors on robot
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);
        extender.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        arm.setPower(0);
        carousel.setPower(0);
        intake.setPower(0);

        // Run motors without encoders reset them
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set target positions first so that error doesn't occur
        frontRight.setTargetPosition(0);
        frontLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        arm.setTargetPosition(0);
        extender.setTargetPosition(0);

        // set arm to run to position, needs to be done only once to maintain position
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set all motors to brake when power is zero
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    //control arm and extender
    public void moveExtender(int level, double power){
        int target = level;

        extender.setTargetPosition(target);

        extender.setPower(power);

        while(Math.abs(extender.getCurrentPosition() - extender.getTargetPosition()) > 10){
            //lol
        }

        //necessary?
        extender.setPower(0);
    }

    public void moveExtender(int level, double power, boolean wait){
        int target = level;

        extender.setTargetPosition(target);

        extender.setPower(power);

        while((Math.abs(extender.getCurrentPosition() - extender.getTargetPosition()) > 10) && !wait){
            //lol
        }

        //necessary?
        if(!wait){
            extender.setPower(0);
        }
    }

    public void moveArm(int level, double power, LinearOpMode instance, boolean wait){
        int target = level;

        arm.setTargetPosition(target);

        //save wrist
        saveWrist();

        arm.setPower(power);
        if(wait){
            while(Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()) > 20){
                //while arm is moving to position
            }
        }

        //necessary or not?
        //arm.setPower(0);
    }

    //automatic driving
    //drive and stop with no reversing
    public void driveToPosition(int distance, double power, LinearOpMode instance, boolean wait){

        power *= 0.85;

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int d = (int) (CALIBRATION * (CPR * distance) / CIRC);

        //int distance = (int) CPR;

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setTargetPosition(d);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setTargetPosition(d);

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setTargetPosition(d);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setTargetPosition(d);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        if(wait){
            while(frontRight.isBusy()){

            } // || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy()
        }

        stop();

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //distance = inches to travel
    //TEST THIS
    public void driveAndStop(int distance, double power, LinearOpMode instance, boolean wait){

        power *= 0.85;

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int d = (int) (CALIBRATION * (CPR * distance) / CIRC);

        //int distance = (int) CPR;

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setTargetPosition(d);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setTargetPosition(d);

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setTargetPosition(d);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setTargetPosition(d);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        if(wait){
            while(frontRight.isBusy()){

            } // || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy()
        }

        backLeft.setPower(-power/2);
        frontRight.setPower(-power/2);
        backLeft.setPower(-power/2);
        backRight.setPower(-power/2);

        instance.sleep(50);

        stop();

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    //d -> negative = right positive = left
    public void strafeToPosition(int d, double power, LinearOpMode instance, boolean wait){

        power *= 0.75;

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int distance = (int) ((CPR * d) / CIRC);

        //int distance = (int) CPR;

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setTargetPosition(distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setTargetPosition(-distance);

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setTargetPosition(-distance);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setTargetPosition(distance);

        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

        if(wait){
            while(frontRight.isBusy()){

            }

            stop();
        }

        /*
        frontRight.setPower(-power);

        frontLeft.setPower(power * flip);
        backLeft.setPower(-power * flip);
        backRight.setPower(power * flip);

        instance.sleep(100);

         */

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //rotation group of functions
    public double getAngle() {
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
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    //negative left turn
    //positive right turn
    public void rotate(int degrees, double power, LinearOpMode instance) {
        double  leftPower, rightPower;

        degrees *= -1;

        power = Math.min(0.7, power);

        // restart imu movement tracking.
        //commented this out because of unpredictable rotation while moving in other ways
        //resetAngle();

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
        //swap > and < maybe
        //originally > then <
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            //^ why?
            //while (instance.opModeIsActive() && getAngle() == 0) {}
            while (instance.opModeIsActive() && getAngle() > degrees) {

            }
        }
        else
        {
            // left turn.
            while (instance.opModeIsActive() && getAngle() < degrees) {

            }
        }

        frontLeft.setPower(-leftPower/2.0);
        frontRight.setPower(-rightPower/2.0);
        backLeft.setPower(-leftPower/2.0);
        backRight.setPower(-rightPower/2.0);

        instance.sleep(50);

        stop();

        // wait for rotation to stop.
        //instance.sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //misc functions, level wrist, save wrist, grab, spin, stop, intake, flip
    public void levelWrist(){
        double armRad = 0.0;
        double wristRad = 0.0;
        //turn arm encoder position into arm angle
        armRad = (((double)arm.getCurrentPosition() - armZeroAnglePosition) * Math.PI) / armClicksPerHalfRevolution;

        if(armRad <= Math.PI/2.0){
            wristRad = -armRad;
        }
        else{
            wristRad = -armRad + Math.PI;
        }

        //turn wrist angle into wrist position and set wrist to said position
        wrist.setPosition(((wristRad * wristClicksPerHalfRevolution) / Math.PI) + wristZeroAnglePosition);
    }
    public void saveWrist(){
        wrist.setPosition(safeWrist);
    }
    public void grab(double grabPos){
        grab1.setPosition(grabPos);
    }
    public void spin(double power, boolean direction){
        if(direction){
            carousel.setPower(power);
        }
        else{
            carousel.setPower(-power);
        }
    }
    public void stop() {
        // Set all motors to 0 power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void flip(boolean out){
        if(out){
            //position of the motor when flipper is out
            flipper.setPosition(0.0);
        }
        else{
            //position of the motor when flipper is in
            flipper.setPosition(1.0);
        }
    }

    public void intake(boolean pow){
        //set motor to spin
        if(pow){
            intake.setPower(.8);
        }
        else{
            intake.setPower(0);
        }
    }
}
