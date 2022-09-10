package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Neil Opmode better", group = "TeleOp")
public class NeilCodeV2 extends LinearOpMode{

    //initialize opmode members
    NeilHardwareV2 robot = new NeilHardwareV2();
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
        double driveSpeed = 0.6;
        double carouselSpeed = 0.5;
        double grabPos = 0.45;
        double openPos = 0.55;

        double sens = 0.3;

        //encoder resolution (ticks per rotation)
        //int ppr43 = 3896
        //tick values for the arm postion for levels of the team wobble
        int toptick = 2000;
        int midtick = 2200;
        int bottick = 2400;
        double tickdelta = 0;
        int forwarddrivetick = 200;
        int backwarddrivetick = 1500;
        int ticktolhold = 5;
        int ticktolmove = 10;
        int armlevel = forwarddrivetick;
        double armpowermove = 0.25;
        double armpoweradj = 0.05;

        //control loop
        while(opModeIsActive()){

            forwardleft = driveSpeed*Range.clip(gamepad1.left_stick_y,-1,1);
            strafeleft = -driveSpeed*Range.clip(gamepad1.left_stick_x,-1,1);
            forwardright = driveSpeed*Range.clip(gamepad1.right_stick_y,-1,1);
            straferight = -driveSpeed*Range.clip(gamepad1.right_stick_x, -1, 1);

            // -------------------------------------------------------------
            // -- "always forward" drive direction--------------------------
            // -----code flips the drive controls when arm is raised over
            if (armlevel>=backwarddrivetick){

                //driving reverse orientation - "forward" to place blocks on wobble
                double switchtemp = forwardleft;
                forwardleft = -1*forwardright;
                strafeleft = -1*strafeleft;
                forwardright = -1*switchtemp;
                straferight = -1*straferight;
            }
            //move wheels
            if(Math.abs(forwardleft) > sens || Math.abs(forwardright) > sens || Math.abs(strafeleft) > sens || Math.abs(straferight) > sens){
                robot.setDriveSpeeds(forwardleft, forwardright, strafeleft, straferight, correction);
            }
            else{
                robot.stop();
            }

            //-- control grabber ---------------------------------
            if(gamepad1.dpad_down){
                robot.grab(grabPos);
            } else if(gamepad1.dpad_up) {
                robot.grab(openPos);
            }

            //-- Move arm manually, this was made by andrew for the RSC open house
            if(gamepad1.right_bumper){
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.moveArm(armpowermove);
            }
            else if(gamepad1.left_bumper){
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.moveArm(-armpowermove);
            }
            else{
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.moveArm(0);
            }

            //-- Arm Levels ---------------------------------------
            // 2 safe driving ticks off ground
            // 4 levels for top, middle, bottom levels and intake
            if(armlevel<=forwarddrivetick) {
                if (gamepad1.y || gamepad1.b || gamepad1.x) {
                    armlevel = backwarddrivetick;
                }
                if (gamepad1.a) {
                    armlevel = 0;
                }
            } else if(armlevel>=backwarddrivetick){
                if(gamepad1.y) {
                    armlevel = toptick;
                }
                if(gamepad1.b) {
                    armlevel = midtick;
                }
                if(gamepad1.x) {
                    armlevel = bottick;
                }
                if(gamepad1.a) {
                    armlevel = forwarddrivetick;
                }
            }

            //update postion of arm
            /* commented out by andrew for RSC open house
            robot.arm.setTargetPosition(armlevel);
            //adjust power to move to and maintain postions
            tickdelta = Math.abs((robot.arm.getCurrentPosition()-armlevel));
            if(tickdelta <= ticktolhold){
                robot.arm.setPower(0);
            } else if(tickdelta <= ticktolmove){
                robot.arm.setPower(armpoweradj);
            } else {
                robot.arm.setPower(armpowermove);
            }

             */
        }
    }
}