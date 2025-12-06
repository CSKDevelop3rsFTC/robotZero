package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Meet2TeleOp", group="Robot")
public class Meet2TeleOp extends LinearOpMode {

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;

    public DcMotor intakeMotor = null;

    public DcMotor turretMotor = null;

    public Servo   kickerServo = null;
    public Servo   stopperServo = null;

    public ServoImplEx leftLight  = null;
    public ServoImplEx rightLight = null;
    public ServoImplEx top = null;

    public GoBildaPinpointDriver pinpoint;

    public IMU imu;

    public boolean lightBool;
    public boolean shooterBool;

    // for toggle logic
    private boolean prevLeftBumper = false;

    private boolean turretHold = false;

    public int pos;

    public double counterDrift;

    // Shooter PID subsystem (newTuner-style)
    private ShooterPID shooter;

    private static final double MOTOR_CPR        = 537.6;      // GoBILDA 312RPM CPR at output shaft
    private static final int    GEAR_INPUT_TEETH = 63;      // motor gear
    private static final int    GEAR_OUTPUT_TEETH= 190;     // lazy susan gear

    // motor turns per 1 turret turn = output/input
    private static final double GEAR_RATIO       = (double) GEAR_OUTPUT_TEETH / GEAR_INPUT_TEETH; // 190/63 ≈ 3.01587
    private static final double TICKS_PER_TURRET_REV = MOTOR_CPR * GEAR_RATIO;                    // ≈ 84.444...
    private static final double TICKS_PER_DEG    = TICKS_PER_TURRET_REV / 360.0;                  // ≈ 0.234568
    private static final double DEG_PER_TICK     = 1.0 / TICKS_PER_DEG;                           // ≈ 4.262 deg/tick

    // simple position controller
    private static final double TURRET_KP        = 0.01;   // start small; raises or lowers snappiness
    private static final double TURRET_HOLD      = 0.05;   // gentle hold
    private static final double TURRET_DEG_TOL   = 2.0;    // within 2° considered on target

    private final ElapsedTime ledBlinkTimer = new ElapsedTime();
    private boolean whiteOn = false;

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
        shooter.setTelemetry(
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
        );

        intakeMotor     = hardwareMap.get(DcMotor.class, "intake");

        turretMotor     = hardwareMap.get(DcMotor.class, "turret");

        kickerServo     = hardwareMap.get(Servo.class, "kicker");
        stopperServo    = hardwareMap.get(Servo.class, "stopper");

        leftLight       = hardwareMap.get(ServoImplEx.class, "left");
        rightLight      = hardwareMap.get(ServoImplEx.class, "right");
        top      = hardwareMap.get(ServoImplEx.class, "top");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLight.setPwmEnable();
        rightLight.setPwmEnable();

        lightBool = false;
        shooterBool = false;
        prevLeftBumper = false;

        // safe defaults
        kickerServo.setPosition(0.5);
        stopperServo.setPosition(0.5);

        pos = 0;

        counterDrift = 0;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,  // <— must match physical
                        RevHubOrientationOnRobot.UsbFacingDirection.UP       // <— must match physical
                )
        );

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(imuParams);
        pinpoint.resetPosAndIMU();

        ledBlinkTimer.reset();
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

    private void holdTurretRobotZero() {
        driveTurretToDeg(0.0);
    }

    @Config
    public static class Goal {
        // set these to your real field coordinates for the far goal
        public static double X =  72.0;  // inches example
        public static double Y =  68.0 - 48;  // inches example
        public static double TURRET_MAX_POWER = 0.3;   // keep wiring safe
    }

    // Wrap an angle to [-180, 180]
    private static double wrapDeg(double a){
        while(a > 180)  a -= 360;
        while(a < -180) a += 360;
        return a;
    }

    // Field angle from robot to goal, in degrees
// robotX, robotY are your odometry in field coordinates
    private static double angleToGoalDeg(double robotX, double robotY){
        double dx = Goal.X - robotX;
        double dy = Goal.Y - robotY;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private static double distanceToGoal(double robotX, double robotY) {
        double dx = Goal.X - robotX;   // vertical (field) axis by your naming
        double dy = Goal.Y - robotY;   // horizontal (field) axis by your naming
        return Math.hypot(dx, dy);
    }

    // Convenience overload using currentPose
    private double distanceToGoal() {
        return distanceToGoal(currentPose.x, currentPose.y);
    }

    // within ±25 RPM of target?
    private boolean isShooterAtSpeed() {
        double target = shooter.getTargetRPM();
        if (target < 100) return false;
        return Math.abs(shooter.getCurrentRPM() - target) <= 75.0;
    }

    // Blink white when at speed (toggle ~10 Hz here; change 100 → 200 for slower)
    private void updateBlinkWhiteIfAtSpeed() {
        if (isShooterAtSpeed()) {
            if (ledBlinkTimer.milliseconds() >= 100) {
                whiteOn = !whiteOn;
                lightControl(whiteOn ? "white" : "null");
                ledBlinkTimer.reset();
            }
        }
    }


    // Convert a turret command in degrees to encoder ticks (0° = turret zero)
    private static int degToTicks(double deg){
        return (int) Math.round(deg * TICKS_PER_DEG);
    }

    // Basic closed-loop to a desired turret angle (deg), returns true when on target
    private boolean driveTurretToDeg(double targetTurretDeg){
        // Current turret angle estimate from encoder
        int    ticks = turretMotor.getCurrentPosition();
        double turretDegNow = ticks * DEG_PER_TICK;
        double errDeg   = wrapDeg(targetTurretDeg - turretDegNow);

        if (Math.abs(errDeg) <= 0.5) {
            turretMotor.setPower(0.0);
            return true;
        }

        double cmd      = TURRET_KP * (errDeg * TICKS_PER_DEG); // proportional on ticks
        cmd = Math.max(-Goal.TURRET_MAX_POWER, Math.min(Goal.TURRET_MAX_POWER, cmd));

        // gentle hold bias so it never fully coasts
        if (Math.abs(cmd) < TURRET_HOLD) cmd = Math.signum(cmd) * TURRET_HOLD;

        turretMotor.setPower(cmd);
        return Math.abs(errDeg) <= TURRET_DEG_TOL;

    }

    // Hold field-absolute 0°: turret angle = -robotYaw
    private void holdTurretFieldZero(){
        double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetTurretDeg = wrapDeg(-yawDeg);
        driveTurretToDeg(targetTurretDeg);
    }

    // Aim at goal given robot pose (robotHeadingDeg only used for telemetry here;
// turret command is field-angle-to-goal minus robot yaw)
    private boolean aimTurretAtGoal(double robotX, double robotY){
        double yawDeg   = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double fieldAim = angleToGoalDeg(robotX, robotY);
        double turretDegCmd = wrapDeg(fieldAim - yawDeg - counterDrift);
        return driveTurretToDeg(turretDegCmd);
    }

    @Configurable
    @Config
    public static class transferConstants {
        public static int counterConstant = 3;

        public static double intakePower = 1;

        public static long firstWait     = 225;
        public static long waitAfterKick = 200;
        public static long pauseWait     = 200;
        public static long pauseWait2    = 200;
    }

    public void driveController() {
        if (gamepad1.right_stick_button) driveMovement(0.5);
        else driveMovement(1.0);
    }

    public void intake() {
        if (gamepad1.right_trigger > 0)      intakeMotor.setPower(1);
        else if (gamepad1.left_trigger > 0)  intakeMotor.setPower(-1);
        else if (gamepad1.left_stick_y > 0)  intakeMotor.setPower(gamepad1.left_stick_y);
        else                                 intakeMotor.setPower(0);
    }

    // toggle shooter on/off with left bumper
    public void shooterGamepad() {
        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLeftBumper) {
            shooterBool = !shooterBool;
        }
        prevLeftBumper = lb;
    }

    // manual shooter control (only when NOT transferring)
    public void shooterControl() {
        if (shooterBool) {
            shooter.setTargetRPM( ( (distanceToGoal()) * 5.7992) + 865.091);
        } else {
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

    // "sleep" while keeping the shooter PIDF running
    private void sleepWithShooter(long millis) {
        long end = System.currentTimeMillis() + millis;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            shooter.update();
            aimTurretAtGoal(currentPose.x, currentPose.y);   // keep aiming while we “wait”
            updateBlinkWhiteIfAtSpeed();                     // << add this line
            telemetry.update();
        }
    }



    // Smallest signed difference from current heading to target, in degrees
    private double turretTurnDegrees(double targetHeadingDeg) {
        double current = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // SDK yaw
        double err = targetHeadingDeg - current;
        // wrap into [-180, 180]
        while (err > 180)  err -= 360;
        while (err < -180) err += 360;
        return err;
    }


    public void transfer() {
        if (gamepad1.right_bumper) {

            // 1) Aim at goal
            long aimTimeoutMs = 1500;
            long end = System.currentTimeMillis() + aimTimeoutMs;
            boolean onTarget = false;
            while (opModeIsActive() && System.currentTimeMillis() < end) {
                // keep shooter PID running
                shooter.update();

                // drive turret to target
                onTarget = aimTurretAtGoal(currentPose.x, currentPose.y);

                // simple exit once within tolerance for a short dwell
                if (onTarget) break;
                sleep(10);
            }

            // 2) Open and fire sequence (your existing timings)
            stopperServo.setPosition(0);
            intakeMotor.setPower(transferConstants.intakePower);
            sleepWithShooter(transferConstants.firstWait);
            intakeMotor.setPower(0);

            sleepWithShooter(transferConstants.pauseWait);

            intakeMotor.setPower(transferConstants.intakePower);
            sleepWithShooter(transferConstants.pauseWait2);

            kickerServo.setPosition(0);
            sleepWithShooter(transferConstants.waitAfterKick);
            kickerServo.setPosition(0.5);
            stopperServo.setPosition(0.5);
            intakeMotor.setPower(0);

            // 3) Return to field-absolute 0° the same way we came
            long backTimeoutMs = 1500;
            end = System.currentTimeMillis() + backTimeoutMs;
            while (opModeIsActive() && System.currentTimeMillis() < end) {
                shooter.update();
                holdTurretRobotZero();  // drives back toward 0° field
                sleep(10);
            }
            // keep a gentle hold
            turretMotor.setPower(TURRET_HOLD * Math.signum(turretMotor.getPower()));
            whiteOn = false;
            counterDrift += transferConstants.counterConstant;
            lightControl("blue");
        }
    }


    public void lightControl(String color) {
        if ("null".equalsIgnoreCase(color)) {
            leftLight.setPosition(0);
            rightLight.setPosition(0);
            top.setPosition(0);
        } else if ("red".equalsIgnoreCase(color)) {
            leftLight.setPosition(0.28);
            rightLight.setPosition(0.28);
            top.setPosition(0.28);
        } else if ("blue".equalsIgnoreCase(color)) {
            leftLight.setPosition(0.57);
            rightLight.setPosition(0.57);
            top.setPosition(0.57);
        } else if ("white".equalsIgnoreCase(color)) {
            leftLight.setPosition(1.0);
            rightLight.setPosition(1.0);
            top.setPosition(1.0);
        }
    }


    // =========================
    //  CORNER LOCALIZATION
    // =========================

    // simple pose struct you can use for turret math, auto aiming, etc.
    public static class Pose {
        public double x;        // inches (or mm, your choice)
        public double y;
        public double heading;  // radians

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // assume 144" x 144" field, origin at field center:
    // x+ to your right looking out from BLUE driver station,
    // y+ away from BLUE driver station toward RED.
    private static final double HALF_FIELD = 72.0; // inches

    // BLUE alliance left corner by their driver station
    private static final Pose BLUE_CORNER_POSE =
            new Pose(-HALF_FIELD, -HALF_FIELD, Math.toRadians(0));

    // RED alliance right corner by their driver station
    private static final Pose RED_CORNER_POSE =
            new Pose(HALF_FIELD, HALF_FIELD, Math.toRadians(180));

    // current belief of robot pose on the field
    private Pose currentPose = new Pose(-63, 0, 0);

    // call this when you KNOW you’re in the blue corner
    public void localizeAtBlueCorner() {
        currentPose = new Pose(
                BLUE_CORNER_POSE.x,
                BLUE_CORNER_POSE.y,
                BLUE_CORNER_POSE.heading
        );
    }

    // call this when you KNOW you’re in the red corner
    public void localizeAtRedCorner() {
        currentPose = new Pose(
                RED_CORNER_POSE.x,
                RED_CORNER_POSE.y,
                RED_CORNER_POSE.heading
        );
    }

    // "callable values" to grab your constant field positions
    public Pose getBlueCornerPose() {
        return BLUE_CORNER_POSE;
    }

    public Pose getRedCornerPose() {
        return RED_CORNER_POSE;
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        lightControl("blue");
        imu.resetYaw();

        while (opModeIsActive()) {
            driveController();

            try {
                pinpoint.update();
                Pose2D p = pinpoint.getPosition();
                currentPose.x = p.getX(DistanceUnit.INCH);  // or MM, but be consistent with Goal.X/Y
                currentPose.y = p.getY(DistanceUnit.INCH);
                // Optional if you want field heading tracked too:
                // currentPose.heading = p.getHeading(AngleUnit.RADIANS);
            } catch (Exception ignored) {}

            shooterGamepad();
            stopper();
            kicker();
            intake();

            double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double fieldAim = angleToGoalDeg(currentPose.x, currentPose.y);
            double turretCmdDeg = wrapDeg(fieldAim - yawDeg);
            telemetry.addData("Yaw deg", "%.1f", yawDeg);
            telemetry.addData("Aim field deg", "%.1f", fieldAim);
            telemetry.addData("Turret cmd deg", "%.1f", turretCmdDeg);
            telemetry.addData("x", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Turret ticks", turretMotor.getCurrentPosition());
            telemetry.addData("Dist to goal (in)", "%.2f", distanceToGoal());

            // run transfer sequence if RB is pressed (this internally updates shooter)
            if (gamepad1.right_bumper) {
                transfer();                  // transfer will aim continuously now
            } else {
                holdTurretRobotZero();

                if (isShooterAtSpeed()) {
                    updateBlinkWhiteIfAtSpeed();
                } else {
                    whiteOn = false;
                    lightControl("blue");
                    ledBlinkTimer.reset();
                }
            }

            // normal shooter control + PIDF update when NOT in transfer
            if (!gamepad1.right_bumper) {
                shooterControl();
                shooter.update();   // matches ShooterPID from newTuner
            }

            //telemetry.addData("Shooter Target", shooter.getTargetRPM());
            //telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
            telemetry.update();

            sleep(20);
        }

        shooter.stop();
    }
}
