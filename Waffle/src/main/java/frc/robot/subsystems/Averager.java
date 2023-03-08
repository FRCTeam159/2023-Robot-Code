
package frc.robot.subsystems;

import java.util.LinkedList;

public class Averager {
    private LinkedList<Double> vals = new LinkedList<Double>();
    double total = 0;
    int aves;

    public Averager(int num_aves) {
        aves = num_aves;
    }

    public void reset() {
        vals.clear();
        total=0;
    }

    public int numAves(){
        return aves;
    }

    public double getAve(double d) {
        if (vals.size() == aves)
            total -= vals.removeFirst().doubleValue();
        vals.addLast(d);
        total += d;
        return total / vals.size();
    }
   
}
