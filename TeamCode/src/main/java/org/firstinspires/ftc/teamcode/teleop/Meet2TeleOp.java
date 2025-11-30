package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="Meet2TeleOp", group="Robot")
public class Meet2TeleOp extends LinearOpMode {

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;

    public DcMotor intakeMotor = null;

    public Servo   kickerServo = null;
    public Servo   stopperServo = null;

    public ServoImplEx leftLight  = null;
    public ServoImplEx rightLight = null;

    public boolean lightBool;

    // Shooter PID subsystem
    private ShooterPID shooter;

    Gamepad.LedEffect red = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 250)
            .build();

    Gamepad.LedEffect blue = new Gamepad.LedEffect.Builder()
            .addStep(0.12, 0.32, 1, 250)
            .build();

    Gamepad.LedEffect white = new Gamepad.LedEffect.Builder()
            .addStep(1, 1, 1, 250)
            .build();

    public void initialize() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        // ShooterPID owns f1M and f2M
        shooter = new ShooterPID(hardwareMap, "f1M", "f2M");

        intakeMotor     = hardwareMap.get(DcMotor.class, "intake");

        kickerServo     = hardwareMap.get(Servo.class, "kicker");
        stopperServo    = hardwareMap.get(Servo.class, "stopper");

        leftLight       = hardwareMap.get(ServoImplEx.class, "left");
        rightLight      = hardwareMap.get(ServoImplEx.class, "right");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLight.setPwmEnable();
        rightLight.setPwmEnable();

        lightBool = false;
    }

    public void driveMovement(double driveSpeed) {
        double horizontal = gamepad1.left_stick_x;
        double vertical   = -gamepad1.left_stick_y;
        double pivot      = -gamepad1.right_stick_x * 0.5;

        frontRightDrive.setPower(((vertical + horizontal) + pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal) - pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal) + pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal) - pivot) * driveSpeed);
    }

    @Configurable
    public static class transferConstants {
        public static long initialWait   = 375;
        public static long postKickWait  = 250;
        public static double intakePower = 0.75;
        public static long rpmConstant   = 3200;
        public static long chargeUp      = 1000;
        public static long firstScale      = 1000;
        public static long secondScale      = 2000;
    }

    public void driveController() {
        if (gamepad1.right_stick_button) driveMovement(0.5);
        else driveMovement(1.0);
    }

    public void intake() {
        if (gamepad1.right_trigger > 0)      intakeMotor.setPower(1);
        else if (gamepad1.left_trigger > 0)  intakeMotor.setPower(-1);
        else                                 intakeMotor.setPower(0);
    }

    // manual shooter control (only when NOT transferring)
    public void shooterControl() {
        if (gamepad1.left_bumper) {
            shooter.setTargetRPM(transferConstants.rpmConstant);
        } else if (!gamepad1.right_bumper) {
            // only zero if we are NOT running the transfer sequence
            shooter.setTargetRPM(0.0);
        }
    }

    public void kicker() {
        if (gamepad1.x) kickerServo.setPosition(0);
        if (gamepad1.b) kickerServo.setPosition(0.5);
    }

    public void stopper() {
        if (gamepad1.y) stopperServo.setPosition(0.5);
        if (gamepad1.a) stopperServo.setPosition(0);
    }

    // helper: "sleep" but keep updating shooter so it can accelerate/recover
    // Sleep while still updating shooter PIDF, so RPM can actually change
    private void sleepWithShooter(long millis) {
        double lastTime = getRuntime();
        long end = System.currentTimeMillis() + millis;

        while (opModeIsActive() && System.currentTimeMillis() < end) {
            double now = getRuntime();
            double dt  = Math.max(0.001, now - lastTime);
            lastTime = now;

            shooter.update(dt);

            telemetry.addData("Shooter Target", shooter.getTargetRPM());
            telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
            telemetry.update();
        }
    }


    public void transfer() {
        if (gamepad1.right_bumper) {

            long shotInterval = transferConstants.initialWait; // same spacing for all 3

            // 1) spin up to base RPM
            shooter.setTargetRPM(transferConstants.rpmConstant);
            sleepWithShooter(transferConstants.chargeUp);

            // 2) open gate + start intake
            stopperServo.setPosition(0); // open gate
            intakeMotor.setPower(transferConstants.intakePower);

            // ---- BALL 1 (base RPM) ----
            sleepWithShooter(shotInterval);

            // ---- BALL 2 (first RPM bump) ----
            shooter.setTargetRPM(transferConstants.rpmConstant + transferConstants.firstScale);
            sleepWithShooter(shotInterval);

            // ---- BALL 3 (second bump + kicker) ----
            shooter.setTargetRPM(transferConstants.rpmConstant + transferConstants.secondScale);
            sleepWithShooter(shotInterval);

            kickerServo.setPosition(0); // fire third ball
            sleepWithShooter(transferConstants.postKickWait);
            kickerServo.setPosition(0.5);

            // cleanup
            intakeMotor.setPower(0);
            stopperServo.setPosition(0.5);

            // after auto-sequence, spin down
            shooter.setTargetRPM(0);
        }
    }


    public void lightControl() {
        if (gamepad2.b) {
            gamepad1.runLedEffect(red);
        }
        if (gamepad2.x) {
            gamepad1.runLedEffect(blue);
        }
        if (gamepad2.y) {
            gamepad1.runLedEffect(white);
        }
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        double last = getRuntime();

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt  = Math.max(0.001, now - last);
            last = now;

            driveController();
            lightControl();
            stopper();
            kicker();
            intake();

            // run transfer sequence if RB is pressed (this will internally call shooter.update)
            transfer();

            // normal shooter control + PIDF update when NOT in transfer
            if (!gamepad1.right_bumper) {
                shooterControl();
                shooter.update(dt);
            }

            telemetry.addData("Shooter Target", shooter.getTargetRPM());
            telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
            telemetry.update();

            sleep(20);
        }

        shooter.stop();
    }
}
