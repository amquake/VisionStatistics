package frc.robot.statistics;

import java.util.TreeMap;

public class TimedBuffer<T> extends TreeMap<Double, T> {
    private double bufferSeconds;

    public TimedBuffer() {
        this.bufferSeconds = 3;
    }

    public TimedBuffer(double bufferSeconds) {
        this.bufferSeconds = bufferSeconds;
    }

    public void setBufferSeconds(double bufferSeconds) {
        this.bufferSeconds = bufferSeconds;
    }

    private void cleanup(double newTime) {
        while (!isEmpty()) {
            double first = firstKey();
            if (newTime - first > bufferSeconds) remove(first);
            else return;
        }
    }

    public T update(T value, double timestamp) {
        if (value == null) return null;
        var result = put(timestamp, value);
        cleanup(timestamp);
        return result;
    }
}
