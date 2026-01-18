package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Kinematics;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.google.common.hash.Hashing;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;

/** This is the ErrorCalculator.
 * It is in charge of taking the Poses and Velocity produced by the PoseTracker and determining and returning the errors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class ErrorCalculator {

    private static class CachedErrors {
        long poseHash;
        Vector translationalError;
        double headingError;
        double driveError;
    }

    private final LinkedHashMap<Long, CachedErrors> errorCache = new LinkedHashMap<Long, CachedErrors>(32, 0.75f, true) {
        @Override
        protected boolean removeEldestEntry(Map.Entry<Long, CachedErrors> eldest) {
            return size() > 20;  // LRU кэш на 20 записей
        }
    };

    private FollowerConstants constants;
    private KalmanFilter driveKalmanFilter;
    private Pose closestPose, currentPose;
    private Path currentPath;
    private PathChain currentPathChain;
    private boolean followingPathChain;
    private double[] driveErrors;
    private int chainIndex;
    private double rawDriveError, previousRawDriveError, headingError, xVelocity, yVelocity;
    private Double driveError;
    private Vector velocityVector = new Vector();
    private double headingGoal;
    private long lastTranslationalHash = 0;
    private long lastHeadingHash = 0;

    public ErrorCalculator(FollowerConstants constants) {
        this.constants = constants;

        KalmanFilterParameters driveKalmanFilterParameters = new KalmanFilterParameters(
                10,
                1);

        driveKalmanFilter = new KalmanFilter(driveKalmanFilterParameters);
        driveErrors = new double[2];
    }


    private long hashPoseAndGoal(Pose pose, double goal) {
        return Hashing.murmur3_128().newHasher()
                .putDouble(Math.round(pose.getX() * 100) / 100.0)
                .putDouble(Math.round(pose.getY() * 100) / 100.0)
                .putDouble(Math.round(pose.getHeading() * 1000) / 1000.0)
                .putDouble(Math.round(goal * 1000) / 1000.0)
                .hash().asLong();
    }

    private long hashTranslationalState(Pose current, Pose closest) {
        return Hashing.murmur3_128().newHasher()
                .putDouble(Math.round(current.getX() * 100) / 100.0)
                .putDouble(Math.round(current.getY() * 100) / 100.0)
                .putDouble(Math.round(closest.getX() * 100) / 100.0)
                .putDouble(Math.round(closest.getY() * 100) / 100.0)
                .hash().asLong();
    }

    private long hashDriveState(double distanceToGoal, double velocityMagnitude, double headingDiff) {
        return Hashing.murmur3_128().newHasher()
                .putDouble(Math.round(distanceToGoal * 100) / 100.0)
                .putDouble(Math.round(velocityMagnitude * 100) / 100.0)
                .putDouble(Math.round(headingDiff * 1000) / 1000.0)
                .hash().asLong();
    }

    public void update(Pose currentPose, Path currentPath, PathChain currentPathChain,
                       boolean followingPathChain, Pose closestPose, Vector velocity,
                       int chainIndex, double xMovement, double yMovement, double headingGoal) {
        this.currentPose = currentPose;
        this.velocityVector = velocity;
        this.currentPath = currentPath;
        this.closestPose = closestPose;
        this.currentPathChain = currentPathChain;
        this.followingPathChain = followingPathChain;
        this.chainIndex = chainIndex;
        this.xVelocity = xMovement;
        this.yVelocity = yMovement;
        this.headingGoal = headingGoal;
        driveError = null;  // Инвалидируем кэш drive ошибки
    }

    /**
     * This returns the raw translational error, or how far off the closest point the robot is.
     *
     * @return This returns the raw translational error as a Vector.
     */
    public Vector getTranslationalError() {
        if (closestPose == null || currentPose == null) return new Vector();

        long hash = hashTranslationalState(currentPose, closestPose);


        if (hash == lastTranslationalHash && errorCache.containsKey(hash)) {
            CachedErrors cached = errorCache.get(hash);
            if (cached.translationalError != null) {
                return cached.translationalError.copy();
            }
        }


        Vector error = new Vector();
        double x = closestPose.getX() - currentPose.getX();
        double y = closestPose.getY() - currentPose.getY();
        error.setOrthogonalComponents(x, y);


        CachedErrors cached = errorCache.getOrDefault(hash, new CachedErrors());
        cached.poseHash = hash;
        cached.translationalError = error.copy();
        errorCache.put(hash, cached);

        lastTranslationalHash = hash;
        return error;
    }

    /**
     * This returns the raw heading error, or how far off the closest point the robot is.
     *
     * @return This returns the raw heading error as a double.
     */
    public double getHeadingError() {
        if (currentPath == null || currentPose == null) {
            return 0;
        }

        long hash = hashPoseAndGoal(currentPose, headingGoal);


        if (hash == lastHeadingHash && errorCache.containsKey(hash)) {
            CachedErrors cached = errorCache.get(hash);
            return cached.headingError;
        }


        headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), headingGoal) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), headingGoal);


        CachedErrors cached = errorCache.getOrDefault(hash, new CachedErrors());
        cached.poseHash = hash;
        cached.headingError = headingError;
        errorCache.put(hash, cached);

        lastHeadingHash = hash;
        return headingError;
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the Path
     * at some specified deceleration (well technically just some negative acceleration).
     *
     * @return returns the projected velocity.
     */
    private double getDriveVelocityError(double distanceToGoal, double totalDistance) {
        if (currentPath == null) {
            return 0;
        }

        Vector tangent = currentPath.getClosestPointTangentVector().normalize();
        Vector distanceToGoalVector = tangent.times(distanceToGoal);
        Vector velocity = velocityVector.projectOnto(tangent);

        Vector forwardHeadingVector = new Vector(1.0, currentPose.getHeading());
        double forwardVelocity = forwardHeadingVector.dot(velocity);
        double forwardDistanceToGoal = forwardHeadingVector.dot(distanceToGoalVector);
        double forwardacs = Math.signum(forwardDistanceToGoal) *
                Math.sqrt(Math.abs(forwardDistanceToGoal * constants.forwardZeroPowerAcceleration));

        Vector lateralHeadingVector = new Vector(1.0, currentPose.getHeading() - Math.PI / 2);
        double lateralVelocity = lateralHeadingVector.dot(velocity);
        double lateralDistanceToGoal = lateralHeadingVector.dot(distanceToGoalVector);
        double lateracs = Math.signum(lateralDistanceToGoal) *
                Math.sqrt(Math.abs(lateralDistanceToGoal * constants.lateralZeroPowerAcceleration));

        Vector forwardVelocityError = new Vector(forwardacs, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateracs, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = forwardVelocityError.plus(lateralVelocityError);

        previousRawDriveError = rawDriveError;
        rawDriveError = velocityErrorVector.getMagnitude() * Math.signum(velocityErrorVector.dot(tangent));

        if (driveErrors != null && driveErrors.length >= 2) {
            double projection = Kinematics.predictNextLoopVelocity(driveErrors[1], driveErrors[0]);
            driveKalmanFilter.update(rawDriveError - previousRawDriveError, projection);

            // Циклический сдвиг массива
            driveErrors[0] = driveErrors[1];
            driveErrors[1] = driveKalmanFilter.getState();
        }

        return driveKalmanFilter.getState();
    }

    public double getDriveError() {

        if (driveError != null) return driveError;

        if (currentPath == null) {
            driveError = 0.0;
            return 0;
        }

        double distanceToGoal;
        double totalDistance = 0;

        if (!currentPath.isAtParametricEnd()) {
            if (followingPathChain) {
                PathChain.DecelerationType type = currentPathChain.getDecelerationType();
                if (type == PathChain.DecelerationType.GLOBAL) {
                    double remainingLength = 0;

                    if (chainIndex < currentPathChain.size()) {
                        for (int i = chainIndex + 1; i < currentPathChain.size(); i++) {
                            remainingLength += currentPathChain.getPath(i).length();
                        }
                    }

                    distanceToGoal = remainingLength + currentPath.getDistanceRemaining();

                    Vector tangent = currentPath.getClosestPointTangentVector().normalize();
                    Vector forwardTheoreticalHeadingVector = new Vector(1.0, headingGoal);

                    double stoppingDistance = Kinematics.getStoppingDistance(
                            yVelocity + (xVelocity - yVelocity) * forwardTheoreticalHeadingVector.dot(tangent),
                            constants.forwardZeroPowerAcceleration
                    );
                    if (distanceToGoal >= stoppingDistance * currentPath.getBrakingStartMultiplier()) {
                        driveError = -1.0;
                        return -1;
                    }
                } else if ((type == PathChain.DecelerationType.LAST_PATH && chainIndex < currentPathChain.size() - 1) ||
                        type == PathChain.DecelerationType.NONE) {
                    driveError = -1.0;
                    return -1;
                } else {
                    distanceToGoal = currentPath.getDistanceRemaining();
                }
            } else {
                distanceToGoal = currentPath.getDistanceRemaining();
                totalDistance = currentPath.getDistanceTraveled() + currentPath.getDistanceRemaining();
            }
        } else {
            Vector offset = currentPath.getLastControlPoint().minus(currentPose).getAsVector();
            distanceToGoal = currentPath.getEndTangent().dot(offset);
        }


        double velocityMagnitude = velocityVector.getMagnitude();
        double headingDiff = MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), headingGoal);
        long driveHash = hashDriveState(distanceToGoal, velocityMagnitude, headingDiff);

        if (errorCache.containsKey(driveHash)) {
            CachedErrors cached = errorCache.get(driveHash);
            driveError = cached.driveError;
            return driveError;
        }


        driveError = getDriveVelocityError(distanceToGoal, totalDistance);


        CachedErrors cached = errorCache.getOrDefault(driveHash, new CachedErrors());
        cached.driveError = driveError;
        errorCache.put(driveHash, cached);

        return driveError;
    }

    public double getRawDriveError() {
        return rawDriveError;
    }

    public double[] getDriveErrors() {
        return driveErrors;
    }

    public void breakFollowing() {
        driveError = 0.0;
        headingError = 0;
        rawDriveError = 0;
        previousRawDriveError = 0;
        driveErrors = new double[2];
        Arrays.fill(driveErrors, 0);
        driveKalmanFilter.reset();
        errorCache.clear();  // Очищаем все кэши
        lastTranslationalHash = 0;
        lastHeadingHash = 0;
    }

    public void setConstants(FollowerConstants constants) {
        this.constants = constants;
    }

    public String debugString() {
        return "Current Pose: " + (currentPose != null ? currentPose.toString() : "null") + "\n" +
                "Closest Pose: " + (closestPose != null ? closestPose.toString() : "null") + "\n" +
                "Current Path: " + (currentPath != null ? currentPath.toString() : "null") + "\n" +
                "Following Path Chain: " + followingPathChain + "\n" +
                "Chain Index: " + chainIndex + "\n" +
                "Drive Error: " + getDriveError() + "\n" +
                "Heading Error: " + getHeadingError() + "\n" +
                "Raw Drive Error: " + getRawDriveError() + "\n" +
                "Cache Size: " + errorCache.size();
    }
}