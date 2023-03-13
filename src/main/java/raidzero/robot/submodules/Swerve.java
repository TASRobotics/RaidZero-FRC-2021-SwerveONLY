package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.submodules.SwerveModule.TargetPolarityTuple;
import raidzero.robot.utils.JoystickUtils;
import raidzero.robot.wrappers.SendablePigeon;

public class Swerve extends Submodule {

    private enum ControlState {
        OPEN_LOOP, PATHING
    };

    private static Swerve instance = null;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
    }

    private SwerveModule[] modules = new SwerveModule[4];
    private SendablePigeon pigey;

    private Notifier notifier = new Notifier(() -> {
        for (SwerveModule module : modules) {
            if (module != null) {
                module.motor.processMotionProfileBuffer();
                module.rotor.processMotionProfileBuffer();
            }
        }
    });

    private double[] ypr = new double[3];

    private double omega = 0.0;
    // private double targetAngle = 0.0;
    // private double headingError = 0.0;
    private PIDController headingPID;


    // private SwerveModule d;

    // buffer variables
    private double[][] rotV = new double[4][2];
    private double[] totalV = new double[] {0, 0};

    private double lastAngle = 0.0;

    private ControlState controlState = ControlState.OPEN_LOOP;
    private SwerveModule testmodule;

    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        headingPID.reset();
        headingPID.setSetpoint(0.0);
    }

    public void onInit() {
        // Total number of modules in the swerve
        int motorCount = SwerveConstants.SWERVE_IDS.length;

        pigey = new SendablePigeon(0);
        Shuffleboard.getTab(Tab.MAIN).add("Pigey", pigey).withSize(2, 2).withPosition(4, 2);

        // Create and initialize each module
        for (int i = 0; i < motorCount / 2; i++) {
            modules[i] = new SwerveModule();
            modules[i].onInit(SwerveConstants.SWERVE_IDS[2 * i],
                    SwerveConstants.SWERVE_IDS[2 * i + 1], SwerveConstants.INIT_MODULES_DEGREES[i],
                    i + 1);

            /**
             * moduleAngle: the direction of the vector that is +90deg offset from each modules
             * radius vector modulePos: the components of the rotation vector positions { I: j, k
             * II: -j, k III: -j, -k IV: j, -k }
             */
            double moduleAngle = Math.PI / 4 + (Math.PI / 2) * i;
            rotV[i] = new double[] {-Math.sin(moduleAngle), Math.cos(moduleAngle)};
        }

        headingPID = new PIDController(
            SwerveConstants.HEADING_KP, 
            SwerveConstants.HEADING_KI,
            SwerveConstants.HEADING_KD
        );
        headingPID.setTolerance(1);
        headingPID.enableContinuousInput(0, 360);

        zero();
    }

    @Override
    public void update(double timestamp) {
        // Retrive the pigeon's gyro values
        pigey.getYawPitchRoll(ypr);
        
        for (SwerveModule module : modules) {
            module.update(timestamp);
        }
    }

    /**
     * Runs components in the submodule that have continuously changing inputs.
     */
    @Override
    public void run() {
        for (SwerveModule module : modules) {
            module.run();
        }
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        notifier.stop();
        
        totalV = new double[] {0.1, 0};
        
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {
        // targetAngle = 0;
        totalV = new double[] {0.1, 0};

        zeroHeading();
        for (SwerveModule mod : modules) {
            mod.zero();
        }
    }

    public void zeroRotors() {
        for (SwerveModule mod : modules) {
            mod.zeroRotor();
        }
    }

    /**
     * Zeroes the heading of the swerve.
     */
    public void zeroHeading() {
        lastAngle = 0.0;
        pigey.setYaw(0, 20);
        pigey.setFusedHeading(0, 20);
    }

    /**
     * Drives the swerve.
     * 
     * @param vX     velocity in the x direction
     * @param vY     velocity in the y direction
     * @param omegaR angular velocity of the swerve
     */
    public void drive(double vX, double vY, double omegaR) {
        double mag = Math.hypot(vX + (Constants.SQRTTWO * omegaR / 2),
                vY + (Constants.SQRTTWO * omegaR / 2));
        double coef = 1;
        if (mag > 1) {
            coef = 1 / mag;
        }
        for (int i = 0; i < modules.length; i++) {
            totalV[0] = (vX - omegaR * rotV[i][0]);
            totalV[1] = (vY - omegaR * rotV[i][1]);
            modules[i].setVectorVelocity(totalV, coef);
        }
    }

    public void fieldOrientedDrive(double vX, double vY, double rX) {
        // translational adjustment to move w/ respect to the field
        double heading = Math.toRadians(ypr[0]);
        double PIDheading = pigey.getAbsoluteCompassHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double newX = vX * cos + vY * sin;
        double newY = -vX * sin + vY * cos;

        // TODO(jimmy): Absolute turning instead of relative?
        // rotational adjustment to PID to the directed heading
        // if (Math.abs(rX + rY) > 0.01) {
        // targetAngle = Math.toDegrees(Math.atan2(-rX, rY));
        // }
        // headingError = targetAngle - ypr[0];
        // omega = headingPID.calculate(headingError);
        // if (omega > 1)
        // omega = 1;
        // if (omega < -1)
        // omega = -1;

        if (Math.abs(rX) > 0) {
            // for relative control
            omega = rX;
            lastAngle = PIDheading;
        } else {
            System.out.println("Holding PID");
            omega = headingPID.calculate(PIDheading, lastAngle);
        }

        // send new directions to drive
        drive(newX, newY, omega);
    }

    /**
     * Sets the positions of all the rotors on the swerve modules.
     * 
     * @param angle angle in degrees
     * @return tuple class of target angle & motor polarity
     */
    public TargetPolarityTuple[] setRotorPositions(double angle, boolean optimize) {
        TargetPolarityTuple[] targetAngles = new TargetPolarityTuple[4];
        for (int i = 0; i < 4; ++i) {
            targetAngles[i] = modules[i].setRotorPosWithOutputs(angle, optimize);
        }
        return targetAngles;
    }

    /**
     * Returns the position of the rotor on the specified module.
     * 
     * @param moduleId ID of the swerve module
     * @return rotor position in revolution
     */
    public double getModuleRotorPosition(int moduleId) {
        return modules[moduleId].getRotorPosition();
    }

    public void test(XboxController c) {

        if (c.getAButtonPressed()) {
            testmodule = modules[0];
        }
        if (c.getBButtonPressed()) {
            testmodule = modules[1];
        }
        if (c.getXButtonPressed()) {
            testmodule = modules[2];
        }
        if (c.getYButtonPressed()) {
            testmodule = modules[3];
        }
            testmodule.setVectorVelocity(new double[] {JoystickUtils.deadband(-c.getLeftY()),
            JoystickUtils.deadband(c.getLeftX())}, 1);
            // d.setMotorVelocity(JoystickUtils.deadband(c.getRightY())* 40000);
            // d.setRotorPos(JoystickUtils.deadband(c.getLeftY()) * 360 / (4));
        if (c.getLeftTriggerAxis() > 0.5) {
            testmodule.setRotorPos(90);
        }
    }
}
