package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class Auto extends LinearOpMode {

    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;
    public DcMotor shooter = null;
    public DcMotor wobble = null;
    public Servo belt = null;
    public Servo arm = null;

    public Drivetrain Frankie = null;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    public void runOpMode(){
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive_front");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive_back");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_drive_back");

        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        wobble = hardwareMap.get(DcMotor.class, "wobble");
        belt = hardwareMap.get(Servo.class, "belt");
        arm = hardwareMap.get(Servo.class, "arm");

        Drivetrain Frankie = new Drivetrain(this.hardwareMap);  //String FR, String FL, String BR, String BL

        waitForStart();

        Frankie.driveInches(10,DRIVE_SPEED);
        waitUntilDone();
    }

    public void waitUntilDone (){
        while(Frankie.FrontRight.isBusy() || Frankie.FrontLeft.isBusy() || Frankie.BackLeft.isBusy() || Frankie.BackRight.isBusy()){
            sleep(100);
        }
    }
}
