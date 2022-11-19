package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.SleeveDetector;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class ParkAutoBlueRight extends LinearOpMode {
    //    Pose2d START_POSE = new Pose2d(0, 0, 0);
    Robot robot;
    private ElapsedTime timer;
    public void runOpMode() throws InterruptedException {
        Robot robot;
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.lift.setClaw1Pos(0.31);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if(timer.seconds() < 0.8) {
                robot.drive.leftFront.setPower(1);
                robot.drive.rightFront.setPower(-1);
                robot.drive.rightRear.setPower(-1);
                robot.drive.leftRear.setPower(1);
            }else if(timer.seconds() > 0.8 && timer.seconds() < 6) {
                robot.drive.leftFront.setPower(0);
                robot.drive.rightFront.setPower(0);
                robot.drive.rightRear.setPower(0);
                robot.drive.leftRear.setPower(0);
                robot.lift.setTargetHeight(31);
                robot.lift.setArmPos(0.8);
                if (timer.seconds() > 1) {
                    robot.lift.setTargetRotation(-120);
                } else if (timer.seconds() > 3 && timer.seconds() < 5) {
                    robot.lift.setClaw1Pos(0.51);
                } else {
                    robot.lift.setTargetRotation(0);
                    robot.lift.setTargetHeight(-1);
                }
            }else if(timer.seconds() > 6 && timer.seconds() < 6.3) {
                robot.drive.leftFront.setPower(-1);
                robot.drive.rightFront.setPower(1);
                robot.drive.rightRear.setPower(1);
                robot.drive.leftRear.setPower(-1);
            }else if(timer.seconds() > 6.3) {
                robot.drive.leftFront.setPower(0);
                robot.drive.rightFront.setPower(0);
                robot.drive.rightRear.setPower(0);
                robot.drive.leftRear.setPower(0);
            }

            telemetry.addData("time", timer);
            telemetry.update();

        }
    }

//    SleeveDetector detector = new SleeveDetector();
//    SleeveDetection.Color parkingPos = SleeveDetection.Color.MAGENTA;

//    public void runOpMode() {
//        robot = new Robot(telemetry, hardwareMap);
////        detector.init(hardwareMap, telemetry);
//        robot.lift.setArmPos(0.2);
//        robot.lift.setClaw1Pos(0.31);
//        robot.lift.setTargetRotation(robot.lift.getCurrentRotation());
//        telemetry.addData("firstAngle", robot.drive.getOrientation().firstAngle);
//        telemetry.addData("secondAngle", robot.drive.getOrientation().secondAngle);
//        telemetry.addData("thirdAngle", robot.drive.getOrientation().thirdAngle);
//            robot.lift.setArmPos(0.8);
//        //firstAngle turning
//        //secondAngle tipping
//        //thirdAngle is tipping sideways
//
//
//        TrajectorySequence parking1 = robot.drive.trajectorySequenceBuilder(START_POSE)
//                .lineToLinearHeading(new Pose2d(36, 34, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(90)))
//                .build();
//        TrajectorySequence parking2 = robot.drive.trajectorySequenceBuilder(START_POSE)
//                .lineToLinearHeading(new Pose2d(90 , 0, 0))
//                .build();
//        TrajectorySequence parking3 = robot.drive.trajectorySequenceBuilder(START_POSE)
//                .lineToLinearHeading(new Pose2d(36, 34, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(60, 34, Math.toRadians(90)))
//                .build();
//
//        robot.drive.setPoseEstimate(START_POSE);
//
//        // Waiting for start
////        int counter = 0;
////        while (!isStarted() && !isStopRequested()) {
////            parkingPos = detector.getColor();
////            telemetry.addData("Parking position", parkingPos);
////            telemetry.addData("counter", counter);
////            counter++;
////            telemetry.update();
////        }
////
////        // Start is pressed ...
////        /detector.stop();
//        telemetry.addData("oiasjndoajndoijasdoijsadoij","aiosndikawndasdw");
//        telemetry.update();
//        robot.drive.followTrajectorySequenceAsync(parking2);
//
//        while (!isStopRequested() && robot.drive.isBusy() && opModeIsActive()) {
//            telemetry.update();
//            robot.update();
//        }
//    }
}