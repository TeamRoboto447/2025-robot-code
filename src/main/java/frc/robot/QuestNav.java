// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

@SuppressWarnings("unused")
/** Add your docs here. */
public class QuestNav implements PoseProvider {
    private boolean initializedPosition = false;
    private String networkTableRoot = "oculus";
    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private NetworkTable networkTable;
    private Transform3d robotToQuest;
    private Pose3d initPose = new Pose3d();

    private IntegerEntry miso;
    private IntegerPublisher mosi;

    private IntegerSubscriber frameCount;
    private DoubleSubscriber timestamp;
    private FloatArraySubscriber position;
    private FloatArraySubscriber quaternion;
    private FloatArraySubscriber eulerAngles;
    private DoubleSubscriber battery;

    private ChassisSpeeds velocity;
    private Pose3d previousPose;
    private double previousTime;

    public enum QuestCommand {
        RESET(1);

        public final int questRequestCode;

        private QuestCommand(int command) {
            this.questRequestCode = command;
        }

        public int getQuestRequest() {
            return questRequestCode;
        }
    }

    public QuestNav(Transform3d robotToQuest) {
        super();
        this.robotToQuest = robotToQuest;
        setupNetworkTables(networkTableRoot);
    }

    public QuestNav(Transform3d robotToQuest, String networkTableRoot) {
        super();
        this.robotToQuest = robotToQuest;
        this.networkTableRoot = networkTableRoot;
        setupNetworkTables(networkTableRoot);
    }

    public boolean connected() {
        return ((RobotController.getFPGATime() - battery.getLastChange()) / 1000) < 250;
    }

    private void setupNetworkTables(String root) {
        networkTable = networkTableInstance.getTable(root);
        miso = networkTable.getIntegerTopic("miso").getEntry(0);
        mosi = networkTable.getIntegerTopic("mosi").publish();
        frameCount = networkTable.getIntegerTopic("frameCount").subscribe(0);
        timestamp = networkTable.getDoubleTopic("timestamp").subscribe(0.0);
        position = networkTable.getFloatArrayTopic("position").subscribe(new float[3]);
        quaternion = networkTable.getFloatArrayTopic("quaternion").subscribe(new float[4]);
        eulerAngles = networkTable.getFloatArrayTopic("eulerAngles").subscribe(new float[3]);
        battery = networkTable.getDoubleTopic("battery").subscribe(0.0);
    }

    public Translation3d getRawPosition() {
        return new Translation3d(position.get()[2], -position.get()[0], position.get()[2]);
    }

    private Translation3d correctWorldAxis(Translation3d rawPosition) {
        return rawPosition.rotateBy(robotToQuest.getRotation());
    }

    public Rotation3d getRawRotation() {
        float[] euler = eulerAngles.get();
        return new Rotation3d(Degrees.of(euler[2]), Degrees.of(euler[0]), Degrees.of(-euler[1]));
    }

    public Optional<Pose3d> getRobotPose() {
        if (RobotBase.isReal()) {
            return Optional.of(new Pose3d(getPosition(), getRotation()));
        } else {
            return Optional.empty();
        }
    }

    public Translation3d getPosition() {
        return correctWorldAxis(getRawPosition())
                .plus(robotToQuest.getTranslation())
                .plus(robotToQuest.getTranslation().times(-1).rotateBy(getRotation()))
                .plus(initPose.getTranslation());
    }

    public Rotation3d getRotation() {
        // TODO: To support weird rotations/mountings of quest, implement world axis
        // rotation
        return getRawRotation().plus(initPose.getRotation());
    }

    public double getConfidence() {
        if (RobotBase.isReal()) {
            return 0.1;
        } else {
            return Double.MAX_VALUE;
        }
    }

    public double getCaptureTime() {
        return timestamp.getAsDouble();
    }

    public boolean isActive() {
        if (timestamp.get() == 0.0 || RobotBase.isSimulation()) {
            return false;
        }
        return initializedPosition;
    }

    public boolean processQuestCommand(QuestCommand command) {
        if (miso.get() == 99) {
            return false;
        }
        mosi.set(command.getQuestRequest());
        return true;
    }

    private void resetQuestPose() {
        processQuestCommand(QuestCommand.RESET);
    }

    public void resetPose(Pose3d pose) {
        initializedPosition = true;
        initPose = pose;
        resetQuestPose();
    }

    public void resetPose() {
        initializedPosition = true;
        resetQuestPose();
    }

    public void cleanUpQuestCommand() {
        if (miso.get() == 99) {
            mosi.set(0);
        }
    }

    private void updateVelocity() {
        if (previousPose == null) {
            previousPose = getRobotPose().get();
            previousTime = timestamp.get();
            return;
        }
        double currentTime = timestamp.get();
        double deltaTime = currentTime - previousTime;
        if (deltaTime == 0) {
            return;
        }
        velocity = new ChassisSpeeds(
                (getPosition().getX() - previousPose.getTranslation().getX()) / deltaTime,
                (getPosition().getY() - previousPose.getTranslation().getY()) / deltaTime,
                (getRotation().getZ() - previousPose.getRotation().getZ()) / deltaTime);
        previousTime = currentTime;
        previousPose = getRobotPose().get();
    }

    public ChassisSpeeds getVelocity() {
        if (null != velocity) {
            return velocity;
        }
        return new ChassisSpeeds();
    }

    @Override
    public void update() {
        if (RobotBase.isReal()) {
            cleanUpQuestCommand();
            updateVelocity();

            ChassisSpeeds velocity = getVelocity();
            SmartDashboard.putNumberArray(
                    "Velocity",
                    new double[] {
                            velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond
                    });
        }
    }
}