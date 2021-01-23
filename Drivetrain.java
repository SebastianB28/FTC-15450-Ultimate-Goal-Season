package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Drivetrain{
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: GoBilda (19.2:1 Ratio) Motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // 1:1 gear ration on GoBilda Mecanum Chassis
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // Diameter of GoBilda Mecanum (96mm - 3.77953 in)
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suite the specific robot drive train.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable0

    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;
    BNO055IMU imu;

    public Drivetrain(HardwareMap devices){
        this.FrontRight = devices.get(DcMotor.class, "right_drive_front");
        this.FrontLeft = devices.get(DcMotor.class,"left_drive_front");
        this.BackRight = devices.get(DcMotor.class, "right_drive_back");
        this.BackLeft = devices.get(DcMotor.class, "left_drive_back");

        this.FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.imu = devices.get(BNO055IMU.class, "imu");
        initializeIMU();
    }


    public void driveInches (int inches, double speed){
        int target = inches * (int)COUNTS_PER_INCH;
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + target);
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + target);
        BackRight.setTargetPosition(BackRight.getCurrentPosition() + target);
        BackLeft.setTargetPosition(BackLeft.getCurrentPosition() + target);

        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontRight.setPower(speed);
        FrontLeft.setPower(speed);
        BackRight.setPower(speed);
        BackLeft.setPower(speed);

        //waitUntilDone();
    }

    public void turnToAngle (int angle, double speed){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = angles.firstAngle;
        double targetAngle = yaw + angle;

        if(angle > 0){      //if angle > 0, turn right
            FrontLeft.setPower(speed);
            BackLeft.setPower(speed);

            FrontRight.setPower(-speed);
            BackRight.setPower(-speed);

            while(yaw < targetAngle){
                //sleep(50);
                yaw = angles.firstAngle;
            }

            FrontLeft.setPower(0);
            BackLeft.setPower(0);

            FrontRight.setPower(0);
            BackRight.setPower(0);
        }
        else if(angle < 0){     //if angle < 0, turn left
            FrontLeft.setPower(-speed);
            BackLeft.setPower(-speed);

            FrontRight.setPower(speed);
            BackRight.setPower(speed);

            while(yaw > targetAngle){
                //sleep(50);
                yaw = angles.firstAngle;
            }

            FrontLeft.setPower(0);
            BackLeft.setPower(0);

            FrontRight.setPower(0);
            BackRight.setPower(0);
        }
    }

    public void initializeIMU(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        this.imu.initialize(parameters);
    }

    //public void waitUntilDone(){
        //while(BackLeft.isBusy() || BackRight.isBusy() || FrontRight.isBusy() || FrontLeft.isBusy()){
            //sleep(50);
        //}
    //}
}
