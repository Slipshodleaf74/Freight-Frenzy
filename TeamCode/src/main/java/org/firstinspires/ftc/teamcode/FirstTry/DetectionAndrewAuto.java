package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Detection Andrew Auto")
public class DetectionAndrewAuto extends LinearOpMode{
    //row, column (y, x)
    int[] subMatZero = {110,110};
    int[] subMatOne = {110,355};
    int[] subMatTwo = {110,600};

    int subMatWidth = 30;
    int subMatHeight = 40;

    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 360; // modify for your camera
    AutoHardware robot = new AutoHardware();
    OpenCvWebcam webcam;
    FreightFrenzyPipeline pipeline;

    int position = 0;

    @Override
    public void runOpMode(){
        //initialize robot and camera
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "webcam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new FreightFrenzyPipeline(subMatWidth,subMatHeight,subMatZero,subMatOne,subMatTwo);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        }); //done initializing

        //put telemetry here to output analysis results before starting, while initializing
        while (!isStarted()){
            position = pipeline.position;
            telemetry.addData("Position", position);
            telemetry.update();
        }

        //redundant
        waitForStart();

        //autonomous movements
    }
}
