package frc.robot.statistics;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CornerStatistics extends Statistics<List<TargetCorner>> {
    private final DoublePublisher cornerStdDevXPublisher;
    private final DoublePublisher cornerStdDevYPublisher;

    private final List<TimedBuffer<TargetCorner>> cornerBuffers = new ArrayList<>();
    
    private List<TargetCorner> cornerAvgs = new ArrayList<>();
    private List<TargetCorner> cornerStdDevs = new ArrayList<>();
    private TargetCorner totalCornerSqError = new TargetCorner(0, 0);
    private TargetCorner totalCornerStdDev = new TargetCorner(0, 0);

    public CornerStatistics(String tableName, double bufferSeconds) {
        super(tableName, bufferSeconds);

        var table = NetworkTableInstance.getDefault().getTable(tableName);
        cornerStdDevXPublisher = table.getDoubleTopic("TotalCornerStdDevX").publish();
        cornerStdDevYPublisher = table.getDoubleTopic("TotalCornerStdDevY").publish();
    }

    @Override
    public void setBufferSeconds(double bufferSeconds) {
        super.setBufferSeconds(bufferSeconds);
        if (cornerBuffers != null) for (var buffer : cornerBuffers) buffer.setBufferSeconds(bufferSeconds);
    }

    public void update(double[] tcornxy, double timestamp) {
        var corners = tcornxyToTargetCorners(tcornxy);
        if (corners.size() == 0) return;

        update(corners, timestamp);
    }

    @Override
    public void update(List<TargetCorner> corners, double timestamp) {
        if (corners == null || corners.size() == 0) return;
        
        // clear unused corners
        cornerBuffers.removeIf(b -> timestamp - b.lastKey() > bufferSeconds);

        for (int i = 0; i < corners.size(); i++) {
            if (cornerBuffers.size() <= i) cornerBuffers.add(new TimedBuffer<TargetCorner>(bufferSeconds));
            cornerBuffers.get(i).update(corners.get(i), timestamp);
        }

        calculate();
        publishNT();
    }

    private void calculate() {
        cornerAvgs = new ArrayList<>();
        cornerStdDevs = new ArrayList<>();
        double totalCornerXSqErr = 0;
        double totalCornerYSqErr = 0;
        int totalCorners = 0;
        for (var buffer : cornerBuffers) {
            var corners = buffer.values().toArray(TargetCorner[]::new);
            totalCorners += corners.length;
            var avg = Statistics.findAverage(corners);
            var sqErr = Statistics.findSqErrorSum(avg, corners);
            totalCornerXSqErr += sqErr.x;
            totalCornerYSqErr += sqErr.y;
            var stdDev = new TargetCorner(Math.sqrt(sqErr.x / corners.length), Math.sqrt(sqErr.y / corners.length));
            cornerAvgs.add(avg);
            cornerStdDevs.add(stdDev);
        }
        totalCornerSqError = new TargetCorner(totalCornerXSqErr, totalCornerYSqErr);
        totalCornerStdDev = new TargetCorner(
            Math.sqrt(totalCornerXSqErr / totalCorners),
            Math.sqrt(totalCornerYSqErr / totalCorners)
        );
    }

    private void publishNT() {
        cornerStdDevXPublisher.set(totalCornerStdDev.x);
        cornerStdDevYPublisher.set(totalCornerStdDev.y);
    }

    public List<TargetCorner> getCornerAvgs() {return cornerAvgs;}
    public List<TargetCorner> getCornerStdDevs() {return cornerStdDevs;}
    public TargetCorner getTotalCornerStdDev() {return totalCornerStdDev;}
    public TargetCorner getTotalCornerSqError() {return totalCornerSqError;}

    public static List<TargetCorner> tcornxyToTargetCorners(double[] tcornxy) {
        var corners = new ArrayList<TargetCorner>();
        if (tcornxy == null || tcornxy.length == 0 || tcornxy.length % 2 != 0) return corners;
        
        for (int i = 0; i < tcornxy.length; i+=2) {
            corners.add(new TargetCorner(tcornxy[i], tcornxy[i+1]));
        }
        return corners;
    }

    public static double[] targetCornersToTcornxy(List<TargetCorner> corners) {
        if (corners == null || corners.size() == 0) return new double[]{};
        double[] tcornxy = new double[corners.size()*2];

        for (int i = 0; i < corners.size(); i++) {
            tcornxy[i] = corners.get(i).x;
            tcornxy[i+1] = corners.get(i).y;
        }
        return tcornxy;
    }

    public static List<TargetCorner> allTargetCorners(List<PhotonTrackedTarget> targets) {
        var corners = new ArrayList<TargetCorner>();
        targets.stream().sorted((a,b) -> a.getFiducialId() - b.getFiducialId()).forEach(t -> corners.addAll(t.getDetectedCorners()));
        return corners;
    }
}
