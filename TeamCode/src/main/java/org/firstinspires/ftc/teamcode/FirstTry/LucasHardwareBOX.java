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

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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
public class LucasHardwareBOX {

    /* public op mode members */
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;
    public DcMotorEx arm = null;
    public DcMotorEx carousel = null;
    public DcMotorEx extender = null;
    public DcMotorEx intake = null;

    CRServo intake1 = null;
    CRServo intake2 = null;

    public Servo wrist = null;
    public Servo flipper = null;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    //for arm/wrist/extender maintenance
    private int deltaArm = 0;

    private double extendMax = -1400;

    private double armZeroAnglePosition = 800;
    private double armClicksPerHalfRevolution = 1850;

    private double wristZeroAnglePosition = 0.567;
    private double wristClicksPerHalfRevolution = 0.9 - 0.225;

    private double safeWrist = 1.0;
    private double intakeWrist = .79;

    /* Constructor */
    public LucasHardwareBOX(){
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

        intake1 = hwMap.get(CRServo.class, "Intake1");
        intake2 = hwMap.get(CRServo.class, "Intake2");

        wrist = hwMap.get(Servo.class, "Wrist");
        flipper = hwMap.get(Servo.class, "Flipper");

        // Set motor and servo directions based on orientation of motors on robot
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
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
        extender.setPower(0);
        intake.setPower(0);

        // Run motors without encoders, arm uses encoder, intake doesn't
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

        // set arm to brake when power is zero
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    //manual arm control
    public void moveArm(double power){
        if(arm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        arm.setPower(power);
    }
    public void moveExtender(double power){
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setPower(power);
    }
    //experimental function to set power of arm to maintain its position
    public void maintainArm(){
        //function to change power relative to angle of arm, at positions 400 and 2350 needs to be the highest magnitude (horizontal)
        double angleMultiplier = Math.cos((((double)arm.getCurrentPosition() - armZeroAnglePosition)/armClicksPerHalfRevolution) * Math.PI);
        //function to change power relative to length of extension, min at 0 and max at -1400
        double lengthMultiplier = (Math.abs((double)extender.getCurrentPosition()) / extendMax) + 1.0;

        double coefficient = 0.05;

        arm.setPower(lengthMultiplier * angleMultiplier * coefficient);
    }

    //automatic arm control
    //switch arm to run to position, run to level selected, switch back
    public void encoderExtend(int pos, double power, boolean wait){
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(pos);
        extender.setPower(power);

        while(Math.abs(extender.getCurrentPosition() - extender.getTargetPosition()) > 10 && wait){
            //lol
        }

        if (wait){
            extender.setPower(0);
        }
    }
    public void encoderMoveArm(int pos, double power){
        if(arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        int target = pos;

        arm.setTargetPosition(target);

        //maintain arm if distance to target position gets too big
        deltaArm = Math.abs(arm.getCurrentPosition() - target);

        if(deltaArm <= 10){
            arm.setPower(0);
        }
        else if(deltaArm <= 20){
            arm.setPower(power * 0.05);
        }
        else if(deltaArm <= 40){
            arm.setPower(power * 0.1);
        }
        else if(deltaArm <= 200){
            arm.setPower(power * 0.15);
        }
        else{
            arm.setPower(power);
        }
    }

    //driving
    public void setDriveSpeeds(double forwardleft, double forwardright, double strafeleft, double straferight, double correction) {

        double frontLeftSpeed = forwardleft + strafeleft - correction; //-correction
        double frontRightSpeed = forwardright - straferight + correction; //+correction
        double backLeftSpeed = forwardleft - strafeleft - correction; //-correction
        double backRightSpeed = forwardright + straferight + correction; //+correction

        double largest = 1.0;
        largest = Math.max(largest, Math.abs(frontLeftSpeed));
        largest = Math.max(largest, Math.abs(frontRightSpeed));
        largest = Math.max(largest, Math.abs(backLeftSpeed));
        largest = Math.max(largest, Math.abs(backRightSpeed));

        frontLeft.setPower(frontLeftSpeed / largest);
        frontRight.setPower(frontRightSpeed / largest);
        backLeft.setPower(backLeftSpeed / largest);
        backRight.setPower(backRightSpeed / largest);
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

        //degrees *= -1;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).

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

        // set power to rotate. FOR PIVOT, ONLY POWER FRONT MOTORS
        //frontLeft.setPower(-leftPower);
        //frontRight.setPower(-rightPower);
        backLeft.setPower(-leftPower);
        backRight.setPower(-rightPower);

        // rotate until turn is completed.
        /*
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            //^ why?
            //while (instance.opModeIsActive() && getAngle() == 0) {}

            while (instance.opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (instance.opModeIsActive() && getAngle() < degrees) {}

         */

        //frontLeft.setPower(-leftPower);
        //frontRight.setPower(-rightPower);

    }

    //misc functions, wrist, spin, stop
    public void moveWrist(boolean dir){
        //rate of wrist movement
        double posChange = 0.002;

        if(!dir){
            posChange *= -1.0;
        }

        //change wrist position
        wrist.setPosition(MathUtils.clamp(wrist.getPosition() + posChange,0.0,1.0));
    }
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
    public void intakeWrist(){
        wrist.setPosition(intakeWrist);
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

    public void intake(boolean on, boolean dir){
        //set motor to spin
        if(dir && on){
            intake1.setPower(1.0);
            intake2.setPower(-1.0);
        }
        else if (on){
            intake1.setPower(-1.0);
            intake2.setPower(1.0);
        }
        else{
            intake1.setPower(0.0);
            intake2.setPower(0.0);
        }
    }
}
