package pedroPathing.CompTeleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "FTC30630", group = "Examples")
public class Demo1Player extends OpMode {
    private Follower follower;

    private static final int bankVelocity = 1900;
    private static final int medVelocity = 1900;
    private static final int farVelocity = 2000;
    private static final int maxVelocity = 2450;
    private static final int intakeVelocity = 1400;
    private final Pose startPose = new Pose(0, 0, 0);
    public static DcMotor intake;
    public static DcMotor spinner1;
    private DcMotor shooter1;
    private DcMotor shooter2;



    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);




    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.update();
        intake = hardwareMap.get(DcMotor.class, "intake");
        spinner1 = hardwareMap.get(DcMotor.class, "spinner1");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void init_loop() {/*This method is called continuously after Init while waiting to be started.*/
    }
    @Override
    public void start() {/*This method is called once at the start of the OpMode.*/
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {/*This is the main loop of the opmode and runs continuously after play*/
        follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        /*/Tells you Flywheel Velocity */
        telemetry.addData("Intake Velocity", ((DcMotorEx) intake).getVelocity());
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter1).getVelocity());
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter2).getVelocity());
        telemetry.update();
//----------------------------------------------------------------------------
        /* short shot */
        if (gamepad1.circle) {
            ((DcMotorEx) shooter1).setVelocity(bankVelocity + 1450);/* sets Velocity */
            ((DcMotorEx) shooter2).setVelocity(bankVelocity - 1300);
            if (((DcMotorEx) shooter1).getVelocity() >= bankVelocity - 5) {/* checks Velocity */
                intake.setPower(1);/* Pushes ball to shot */
            } else {
                intake.setPower(0);
}//----------------------------------------------------------------------------
        } /* mid shot */
        else if (gamepad1.cross) {
            ((DcMotorEx) shooter1).setVelocity(medVelocity);
            ((DcMotorEx) shooter2).setVelocity(medVelocity - 640);
            if (((DcMotorEx) shooter1).getVelocity() >= medVelocity - 5) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
}//----------------------------------------------------------------------------
        } /* long shot */
        else if (gamepad1.triangle) {
                ((DcMotorEx) shooter1).setVelocity(farVelocity);
                ((DcMotorEx) shooter2).setVelocity(farVelocity - 500);
                if (((DcMotorEx) shooter1).getVelocity() >= farVelocity - 5) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
}//----------------------------------------------------------------------------
        } /* ? */
        else if (gamepad1.square) {
            ((DcMotorEx) shooter1).setVelocity(maxVelocity);
            ((DcMotorEx) shooter2).setVelocity(bankVelocity);
            if (((DcMotorEx) shooter1).getVelocity() >= maxVelocity - 16) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
}//----------------------------------------------------------------------------
        }/* intake */
        else if (gamepad1.right_bumper) {
            ((DcMotorEx) intake).setVelocity(intakeVelocity -2400);
           if (((DcMotorEx) intake).getVelocity() >= intakeVelocity - 2000) {
            } else {
                intake.setPower(0);
            }
        } else {/* powers down all motors */
            ((DcMotorEx) shooter1).setVelocity(0);
            ((DcMotorEx) shooter2).setVelocity(0);
            shooter1.setPower(0);
            shooter2.setPower(0);
            intake.setPower(0);
//----------------------------------------------------------------------------
            if (gamepad1.right_bumper) {// intakes balls
                intake.setPower(1);}
            else {
                intake.setPower(0);}
//----------------------------------------------------------------------------
            if (gamepad1.left_bumper) {// outtakes balls
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
//----------------------------------------------------------------------------
            if (gamepad1.dpad_left) {
            }
            if (gamepad1.dpad_right) {
            }
            if (gamepad1.dpad_up) {
            }
            if (gamepad1.dpad_down) {
            }
            if (gamepad1.start) {
            }
        }
    }
}


