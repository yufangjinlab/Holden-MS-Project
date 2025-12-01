package org.firstinspires.ftc.teamcode.Bot1.Hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pedro.GoBildaPinpointDriver;

public class DriveTrain {
    HardwareMap hwMap = null;

    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public VoltageSensor battery;
    public double speed, strafe, turn;
    public double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;
    public double leftSpeedAdjust = 1;
    public double rightSpeedAdjust = 1;
    public GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        for (LynxModule hub : hwMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        this.leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        this.rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        this.rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        this.pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftBack.setDirection(DcMotor.Direction.REVERSE);
        this.rightBack.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);

        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftFront.setPower(0);
        this.leftBack.setPower(0);
        this.rightBack.setPower(0);
        this.rightFront.setPower(0);

        this.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.pinpoint.setOffsets(3.25, -6.375, DistanceUnit.INCH); ; //these are tuned for 3110-0002-0001 Product Insight #1

        this.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        this.pinpoint.recalibrateIMU();
//        this.pinpoint.resetPosAndIMU();
    }
}

