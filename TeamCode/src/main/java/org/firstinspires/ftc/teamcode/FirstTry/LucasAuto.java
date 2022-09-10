package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "built different auto")

//test for red carousel side
public class LucasAuto extends LinearOpMode{
    AutoHardware robot = new AutoHardware();

    double driveSpeed = 0.6;    double strafeSpeed = 0.5;
    double rotateSpeed = 0.3;
    double armSpeed = 0.3;
    double carouselSpeed = 0.5;
    double extendSpeed = 0.8;
    double grabPos = 0.8;

    public void telemetry(){
        telemetry.addData("wheel position:", Integer.toString(robot.frontRight.getCurrentPosition()));
        telemetry.addData("arm position:", Integer.toString(robot.arm.getCurrentPosition()));
        telemetry.addData("extender position", Integer.toString(robot.extender.getCurrentPosition()));
        telemetry.addData("angle:", Double.toString(robot.getAngle()));
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);


        //reset grabber
        robot.grab(grabPos);

        //telemetry communication
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        waitForStart();


        //movements for auto
        //strafe to right
        robot.strafeToPosition(-33, strafeSpeed,this, true);

        sleep(500);

        //strafe into wall to align
        robot.driveToPosition(-3,driveSpeed,this, true);

        sleep(500);

        //drive forward
        robot.driveAndStop(12,driveSpeed,this, true);

        sleep(500);

        //rotate to place
        robot.rotate(-180,rotateSpeed,this);

        //move arm up
        robot.moveArm(3,armSpeed,this, true);

        sleep(500);

        //extend arm
        robot.moveExtender(3,extendSpeed);

        //release
        robot.grab(0);

        sleep(200);

        robot.grab(grabPos);

        //retract arm
        robot.moveExtender(0, extendSpeed);

        //move arm back
        robot.moveArm(0, armSpeed, this, true);

        sleep(200);

        //strafe to wall
        robot.strafeToPosition(-60, strafeSpeed,this, true);

        sleep(500);

        //drive up to carousel
        robot.driveToPosition(3,driveSpeed/2,this, true);

        //spin carousel
        robot.spin(carouselSpeed,false);

        //wait
        sleep(2500);

        //stop spinning
        robot.spin(0,true);

        //back up to depot
        robot.driveAndStop(-25, driveSpeed,this, true);

        sleep(500);

        //strafe into wall to park
        robot.strafeToPosition(-3, strafeSpeed,this, true);

        //park
        robot.stop();
    }
}