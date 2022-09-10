package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Detection Test Auto")
public class DetectionTestAuto extends LinearOpMode {
    AutoHardware robot = new AutoHardware();
    private OpenCvCamera webcam;
    private MarkerDetector detector;
    private MarkerDetector.CappingPosition position;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Webcam", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(detector);
        webcam.startStreaming(640, 320, OpenCvCameraRotation.UPRIGHT);
        position = detector.position;

        waitForStart();

        position = detector.position;
        telemetry.addData("position", position);
        telemetry.update();

        //add actual code here
        if (position.equals("LEFT")) {
            telemetry.addData("The robot is doing left stuff ", position);
        }
        if (position.equals("CENTER")) {
            telemetry.addData("The robot is doing center stuff ", position);
        }
        if (position.equals("RIGHT")) {
            telemetry.addData("The robot is doing right stuff ", position);
        }
    }
}
