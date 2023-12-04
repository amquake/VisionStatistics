package frc.robot.statistics;

import java.util.Collection;

import org.ejml.data.DMatrix4;
import org.ejml.data.DMatrix4x4;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.fixed.CommonOps_DDF4;
import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public abstract class Statistics<Input> {
    protected double bufferSeconds;

    public Statistics(String tableName, double bufferSeconds) {
        setBufferSeconds(bufferSeconds);
    }

    public void setBufferSeconds(double bufferSeconds) {
        this.bufferSeconds = bufferSeconds;
    }

    public abstract void update(Input input, double timestampSeconds);


    //////////////////// Static statistics calculations
    //----- Double
    public static double findAverage(Collection<Double> values) {
        if (values == null || values.size() == 0) return 0.0;

        double average = 0;
        for (double v : values) average += v;
        average /= values.size();
        return average;
    }

    public static double findAverage(double... values) {
        if (values == null || values.length == 0) return 0.0;

        double average = 0;
        for (double v : values) average += v;
        average /= values.length;
        return average;
    }

    public static double findSqErrorSum(double avg, Collection<Double> values) {
        if (values == null || values.size() == 0) return 0.0;

        double sum = 0;
        for (double v : values) {
            double diff = v - avg;
            sum += diff*diff;
        }
        return sum;
    }

    public static double findSqErrorSum(double avg, double... values) {
        if (values == null || values.length == 0) return 0.0;

        double sum = 0;
        for (double v : values) {
            double diff = v - avg;
            sum += diff*diff;
        }
        return sum;
    }
    
    public static double findStdDev(double avg, Collection<Double> values) {
        if (values == null || values.size() == 0) return 0.0;

        double sum = findSqErrorSum(avg, values);
        sum /= values.size();
        sum = Math.sqrt(sum);
        return sum;
    }

    public static double findStdDev(double avg, double... values) {
        if (values == null || values.length == 0) return 0.0;

        double sum = findSqErrorSum(avg, values);
        sum /= values.length;
        sum = Math.sqrt(sum);
        return sum;
    }

    //----- Translation
    public static Translation3d findAverage(Translation3d... values) {
        double x = 0;
        double y = 0;
        double z = 0;
        for (var v : values) {
            x += v.getX();
            y += v.getY();
            z += v.getZ();
        }
        return new Translation3d(x/values.length, y/values.length, z/values.length);
    }

    public static Translation3d findSqErrorSum(Translation3d avg, Translation3d... values) {
        if (values == null || values.length == 0) return new Translation3d();

        double x = 0;
        double y = 0;
        double z = 0;
        for (var v : values) {
            double diffX = v.getX() - avg.getX();
            double diffY = v.getY() - avg.getY();
            double diffZ = v.getZ() - avg.getZ();
            x += diffX*diffX;
            y += diffY*diffY;
            z += diffZ*diffZ;
        }
        return new Translation3d(x, y, z);
    }

    public static Translation3d findStdDev(Translation3d avg, Translation3d... values) {
        if (values == null || values.length == 0) return new Translation3d();

        var sum = findSqErrorSum(avg, values);
        double x = sum.getX();
        double y = sum.getY();
        double z = sum.getZ();
        x /= values.length;
        y /= values.length;
        z /= values.length;
        x = Math.sqrt(x);
        y = Math.sqrt(y);
        z = Math.sqrt(z);
        return new Translation3d(x, y, z);
    }

    //----- Rotation
    public static Rotation3d findAverage(Rotation3d... values) {
        // averaging quaternions https://math.stackexchange.com/a/3435296
        DMatrix4x4 rotAccum = new DMatrix4x4();
        DMatrix4 qvec = new DMatrix4();

        for (var v : values) {
            var q = v.getQuaternion();
            qvec.setTo(q.getX(), q.getY(), q.getZ(), q.getW());
            CommonOps_DDF4.multAddOuter(1, rotAccum, 1, qvec, qvec, rotAccum);
        }

        var rotEigDecomp = new SimpleEVD<SimpleMatrix>(new DMatrixRMaj(rotAccum));
        var rotEigVals = rotEigDecomp.getEigenvalues();
        int rotMaxEigValIndex = 0;
        for(int i = 1; i < rotEigVals.size(); i++) {
            if(rotEigVals.get(i).getReal() > rotEigVals.get(rotMaxEigValIndex).getReal()) rotMaxEigValIndex = i;
        }
        var rotMaxEigVec = new Matrix<>(rotEigDecomp.getEigenVector(rotMaxEigValIndex));
        double[] rotMaxEigVecData = rotMaxEigVec.getData();

        return new Rotation3d(new Quaternion(
                rotMaxEigVecData[3],
                rotMaxEigVecData[0],
                rotMaxEigVecData[1],
                rotMaxEigVecData[2]
            ).normalize());
    }

    public static Rotation3d findAverageFast(Rotation3d... values) {
        double xCos = 0;
        double xSin = 0;
        double yCos = 0;
        double ySin = 0;
        double zCos = 0;
        double zSin = 0;
        for (var v : values) {
            xCos += Math.cos(v.getX());
            xSin += Math.sin(v.getX());
            yCos += Math.cos(v.getY());
            ySin += Math.sin(v.getY());
            zCos += Math.cos(v.getZ());
            zSin += Math.sin(v.getZ());
        }
        xCos /= values.length;
        xSin /= values.length;
        yCos /= values.length;
        ySin /= values.length;
        zCos /= values.length;
        zSin /= values.length;
        return new Rotation3d(
            new Rotation2d(xCos, xSin).getRadians(),
            new Rotation2d(yCos, ySin).getRadians(),
            new Rotation2d(zCos, zSin).getRadians());
    }    

    public static double[] findSqErrorSum(Rotation3d avg, Rotation3d... values) {
        if (values == null || values.length == 0) return new double[]{};

        double x = 0;
        double y = 0;
        double z = 0;
        for (var v : values) {
            double diffX = MathUtil.angleModulus(v.getX() - avg.getX());
            double diffY = MathUtil.angleModulus(v.getY() - avg.getY());
            double diffZ = MathUtil.angleModulus(v.getZ() - avg.getZ());
            x += diffX*diffX;
            y += diffY*diffY;
            z += diffZ*diffZ;
        }
        return new double[]{x, y, z};
    }

    public static double[] findStdDev(Rotation3d avg, Rotation3d... values) {
        if (values == null || values.length == 0) return new double[]{};

        var sum = findSqErrorSum(avg, values);
        double x = sum[0];
        double y = sum[1];
        double z = sum[2];
        x /= values.length;
        y /= values.length;
        z /= values.length;
        x = Math.sqrt(x);
        y = Math.sqrt(y);
        z = Math.sqrt(z);
        return new double[]{x, y, z};
    }

    //----- Pose3d
    public static Pose3d findAverage(Pose3d... values) {
        Translation3d[] trls = new Translation3d[values.length];
        Rotation3d[] rots = new Rotation3d[values.length];
        for (int i = 0; i < values.length; i++) {
            trls[i] = values[i].getTranslation();
            rots[i] = values[i].getRotation();
        }
        return new Pose3d(findAverage(trls), findAverage(rots));
    }

    public static Transform3d[] posesToTrfs(Pose3d[] poses) {
        if (poses == null || poses.length == 0) return new Transform3d[]{};
        Transform3d[] trfs = new Transform3d[poses.length];
        for (int i = 0; i < poses.length; i++) {
            trfs[i] = new Transform3d(poses[i].getTranslation(), poses[i].getRotation());
        }
        return trfs;
    }

    public static Pose3d[] trfsToPoses(Transform3d[] trfs) {
        if (trfs == null || trfs.length == 0) return new Pose3d[]{};
        Pose3d[] poses = new Pose3d[trfs.length];
        for (int i = 0; i < trfs.length; i++) {
            poses[i] = new Pose3d(trfs[i].getTranslation(), trfs[i].getRotation());
        }
        return poses;
    }

    //----- TargetCorner
    public static TargetCorner findAverage(TargetCorner... values) {
        double x = 0;
        double y = 0;
        for (var v : values) {
            x += v.x;
            y += v.y;
        }
        return new TargetCorner(x/values.length, y/values.length);
    }

    public static TargetCorner findSqErrorSum(TargetCorner avg, TargetCorner[] values) {
        if (values == null || values.length == 0) return new TargetCorner(0, 0);

        double x = 0;
        double y = 0;
        for (int i = 0; i < values.length; i++) {
            double diffX = values[i].x - avg.x;
            double diffY = values[i].y - avg.y;
            x += diffX*diffX;
            y += diffY*diffY;
        }
        return new TargetCorner(x, y);
    }

    public static TargetCorner findStdDev(TargetCorner avg, TargetCorner[] values) {
        if (values == null || values.length == 0) return new TargetCorner(0, 0);

        var sum = findSqErrorSum(avg, values);
        double x = sum.x;
        double y = sum.y;
        x /= values.length;
        y /= values.length;
        x = Math.sqrt(x);
        y = Math.sqrt(y);
        return new TargetCorner(x, y);
    }
}
