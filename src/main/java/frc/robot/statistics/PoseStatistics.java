package frc.robot.statistics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PoseStatistics extends Statistics<Pose3d> {
    private final DoublePublisher trlAvgXPublisher;
    private final DoublePublisher trlAvgYPublisher;
    private final DoublePublisher trlAvgZPublisher;
    private final DoublePublisher rotAvgXPublisher;
    private final DoublePublisher rotAvgYPublisher;
    private final DoublePublisher rotAvgZPublisher;
    private final DoublePublisher trlStdDevXPublisher;
    private final DoublePublisher trlStdDevYPublisher;
    private final DoublePublisher trlStdDevZPublisher;
    private final DoublePublisher rotStdDevXPublisher;
    private final DoublePublisher rotStdDevYPublisher;
    private final DoublePublisher rotStdDevZPublisher;

    private final TimedBuffer<Translation3d> targetTrlBuffer = new TimedBuffer<>();
    private final TimedBuffer<Rotation3d> targetRotBuffer = new TimedBuffer<>();
    
    private Translation3d translationAvg = new Translation3d();
    private Translation3d translationStdDev = new Translation3d();
    private Rotation3d rotationAvg = new Rotation3d();
    private double[] rotationStdDev = {};

    public PoseStatistics(String tableName, double bufferSeconds) {
        super(tableName, bufferSeconds);

        var table = NetworkTableInstance.getDefault().getTable(tableName);
        trlAvgXPublisher = table.getDoubleTopic("TranslationAverageX").publish();
        trlAvgYPublisher = table.getDoubleTopic("TranslationAverageY").publish();
        trlAvgZPublisher = table.getDoubleTopic("TranslationAverageZ").publish();
        rotAvgXPublisher = table.getDoubleTopic("RotationAverageXDeg").publish();
        rotAvgYPublisher = table.getDoubleTopic("RotationAverageYDeg").publish();
        rotAvgZPublisher = table.getDoubleTopic("RotationAverageZDeg").publish();
        trlStdDevXPublisher = table.getDoubleTopic("TranslationStdDevX").publish();
        trlStdDevYPublisher = table.getDoubleTopic("TranslationStdDevY").publish();
        trlStdDevZPublisher = table.getDoubleTopic("TranslationStdDevZ").publish();
        rotStdDevXPublisher = table.getDoubleTopic("RotationStdDevXDeg").publish();
        rotStdDevYPublisher = table.getDoubleTopic("RotationStdDevYDeg").publish();
        rotStdDevZPublisher = table.getDoubleTopic("RotationStdDevZDeg").publish();
    }

    @Override
    public void setBufferSeconds(double bufferSeconds) {
        super.setBufferSeconds(bufferSeconds);
        if (targetTrlBuffer != null) targetTrlBuffer.setBufferSeconds(bufferSeconds);
        if (targetRotBuffer != null) targetRotBuffer.setBufferSeconds(bufferSeconds);
    }

    @Override
    public void update(Pose3d pose, double timestamp) {
        if (pose == null) return;

        targetTrlBuffer.update(pose.getTranslation(), timestamp);
        targetRotBuffer.update(pose.getRotation(), timestamp);

        calculate();
        publishNT();
    }

    private void calculate() {
        var trls = targetTrlBuffer.values().toArray(Translation3d[]::new);
        translationAvg = Statistics.findAverage(trls);
        translationStdDev = Statistics.findStdDev(translationAvg, trls);

        var rots = targetRotBuffer.values().toArray(Rotation3d[]::new);
        rotationAvg = Statistics.findAverage(rots);
        rotationStdDev = Statistics.findStdDev(rotationAvg, rots);
    }

    private void publishNT() {
        trlAvgXPublisher.set(translationAvg.getX());
        trlAvgYPublisher.set(translationAvg.getY());
        trlAvgZPublisher.set(translationAvg.getZ());
        rotAvgXPublisher.set(Math.toDegrees(rotationAvg.getX()));
        rotAvgYPublisher.set(Math.toDegrees(rotationAvg.getY()));
        rotAvgZPublisher.set(Math.toDegrees(rotationAvg.getZ()));
        trlStdDevXPublisher.set(translationStdDev.getX());
        trlStdDevYPublisher.set(translationStdDev.getY());
        trlStdDevZPublisher.set(translationStdDev.getZ());
        rotStdDevXPublisher.set(Math.toDegrees(rotationStdDev[0]));
        rotStdDevYPublisher.set(Math.toDegrees(rotationStdDev[1]));
        rotStdDevZPublisher.set(Math.toDegrees(rotationStdDev[2]));
    }

    public Translation3d getTranslationAvg() {return translationAvg;}
    public Translation3d getTranslationStdDev() {return translationStdDev;}
    public Rotation3d getRotationAvg() {return rotationAvg;}
    public double[] getRotationStdDev() {return rotationStdDev;}
}
