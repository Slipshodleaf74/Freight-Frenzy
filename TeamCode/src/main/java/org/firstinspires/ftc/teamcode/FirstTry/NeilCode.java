package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Neil Opmode", group = "TeleOp")
public class NeilCode extends LinearOpMode{

    //initialize opmode members
    NeilHardware robot = new NeilHardware();
    private ElapsedTime runtime = new ElapsedTime();

    //initialize
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        */

        waitForStart();
        runtime.reset();

        //variables for controlling
        double forwardleft, forwardright, strafeleft, straferight, armMove;
        double correction = 0; //dunno abt this one, maybe useful for autonomoosssss
        double armSpeed = 0.4;
        double carouselSpeed = 0.5;
        double grabPos = 0.4;
        double openPos = 0.15;

        int armlevel = 0;

        double sens = 0.3;

        //tick values for the arm postion for levels of the team wobble


        //control loop
        while(opModeIsActive()){

            //incredibly scuffed, but it works. the sticks don't correspond to the right variables, but it works
            forwardleft = Range.clip(gamepad1.left_stick_x,-1,1);
            strafeleft = Range.clip(gamepad1.left_stick_y,-1,1);
            forwardright = Range.clip(gamepad1.right_stick_x,-1,1);
            straferight = Range.clip(gamepad1.right_stick_y, -1, 1);

            //armMove = Range.clip(gamepad2.left_stick_y,-1,1);

            //move wheels
            if(Math.abs(forwardleft) > sens || Math.abs(forwardright) > sens || Math.abs(strafeleft) > sens || Math.abs(straferight) > sens){
                robot.setDriveSpeeds(forwardleft, forwardright, strafeleft, straferight, correction);
            }
            else{
                robot.stop();
            }

            /*move arm
            if(Math.abs(armMove) > sens){
                robot.moveArm(armMove * armSpeed);
            }
            else{
                robot.moveArm(0);
            }
            */

            //control grabber
            //if(gamepad2.left_bumper){
            //  robot.grab(grabPos);
            //}
            //else if(gamepad2.right_bumper) {
            //   robot.grab(openPos);
            //}
            //use y button for top level directly
            if(gamepad1.y)
            {
                armlevel = 1;
                robot.ArmPosition(armlevel, 0.2);
            }
            //use a button for top level directly
            if(gamepad1.a)
            {
                armlevel = 0;
                robot.ArmPosition(armlevel, 0.2);
            }

        }
    }
}