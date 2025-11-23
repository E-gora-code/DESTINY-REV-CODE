package org.firstinspires.ftc.teamcode.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class customconsts {
    /** The Forward Velocity of the Robot - Different for each robot
     *  Default Value: 81.34056 */
    public  double xVelocity = 81.34056;

    /** The Lateral Velocity of the Robot - Different for each robot
     *  Default Value: 65.43028 */
    public  double yVelocity = 65.43028;

    private  double[] convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);

    /** The actual drive vector for the front left wheel, if the robot is facing a heading of 0 radians with the wheel centered at (0,0)
     *  Default Value: new Vector(convertToPolar[0], convertToPolar[1])
     * @implNote This vector should not be changed, but only accessed.
     */
    public  Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
    public  double maxPower = 1;
    public  String leftFrontMotorName = "leftFront";
    public  String leftRearMotorName = "leftRear";
    public  String rightFrontMotorName = "rightFront";
    public  String rightRearMotorName = "rightRear";
    public  DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  double MotorPower0 = 1;
    public  double MotorPower1 = 1;
    public  double MotorPower2 = 1;
    public  double MotorPower3 = 1;
    public double MotorsScale = 0.5;


    public  double motorCachingThreshold = 0.01;
    public  boolean useBrakeModeInTeleOp = false;
    public  boolean useVoltageCompensation = false;
    public  double nominalVoltage = 12.0;
    public  double staticFrictionCoefficient = 0.1;


    public customconsts() {
        defaults();
    }

    public customconsts xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public customconsts yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public customconsts maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public customconsts leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }
    public customconsts MotorPower0(double MotorPower0) {
        this.MotorPower0 = MotorPower0;
        return this;
    }
    public customconsts MotorPower1(double MotorPower1) {
        this.MotorPower1 = MotorPower1;
        return this;
    }

    public customconsts MotorsScale(double MotorsScale) {
        this.MotorsScale = MotorsScale;
        return this;
    }
    public customconsts MotorPower2(double MotorPower2) {
        this.MotorPower2 = MotorPower2;
        return this;
    }
    public customconsts MotorPower3(double MotorPower3) {
        this.MotorPower3 = MotorPower3;
        return this;
    }

    public customconsts leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public customconsts rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public customconsts rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public customconsts leftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public customconsts leftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
        return this;
    }

    public customconsts rightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public customconsts rightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
        return this;
    }

    public customconsts motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public customconsts useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public customconsts useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public customconsts nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public customconsts staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public Vector getFrontLeftVector() {
        return frontLeftVector;
    }

    public void setFrontLeftVector(Vector frontLeftVector) {
        this.frontLeftVector = frontLeftVector;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getLeftFrontMotorName() {
        return leftFrontMotorName;
    }

    public void setLeftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
    }

    public String getLeftRearMotorName() {
        return leftRearMotorName;
    }

    public void setLeftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
    }

    public String getRightFrontMotorName() {
        return rightFrontMotorName;
    }

    public void setRightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
    }

    public String getRightRearMotorName() {
        return rightRearMotorName;
    }

    public void setRightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
    }

    public DcMotorSimple.Direction getLeftFrontMotorDirection() {
        return leftFrontMotorDirection;
    }

    public void setLeftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
    }

    public DcMotorSimple.Direction getLeftRearMotorDirection() {
        return leftRearMotorDirection;
    }

    public void setLeftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
    }

    public DcMotorSimple.Direction getRightFrontMotorDirection() {
        return rightFrontMotorDirection;
    }

    public void setRightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
    }

    public DcMotorSimple.Direction getRightRearMotorDirection() {
        return rightRearMotorDirection;
    }

    public void setRightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public boolean isUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    /**
     * This method sets the default values for the MecanumConstants class.
     * It is called in the constructor of the MecanumConstants class.
     */
    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftRear";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightRear";
        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        motorCachingThreshold = 0.01;
        useBrakeModeInTeleOp = false;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
    }
}
