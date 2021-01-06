package frc.robot.util;

import javax.annotation.Nullable;
import java.util.HashMap;

public class Step {
    public enum StepValues {
        command ("command"),
        speed ("speed"),
        distance ("distance"),
        parallel ("parallel"),
        parm ("parm");

        String name = "";

        StepValues(String str) { 
            this.name = str;
        }

        public String toString() { return name; }
    }

    HashMap<String, String> values = new HashMap<String, String>();

    public Step (String _command, double _speed, double _distance, boolean _parallel, double[] _parms) {
        this.values.put(StepValues.command.toString(),  _command);
        this.values.put(StepValues.speed.toString(),  String.valueOf(_speed));
        this.values.put(StepValues.distance.toString(),  String.valueOf(_distance));
        this.values.put(StepValues.parallel.toString(),  String.valueOf(_parallel));
        for (int i = 0; i < _parms.length; i++) {
            this.values.put(StepValues.parm.toString() + i,  String.valueOf(_parms[i]));   
        }
    }

    public Step (String _command, String _speed, String _distance, String _parallel, String[] _parms) {
        this.values.put(StepValues.command.toString(),  _command);
        this.values.put(StepValues.speed.toString(),  (_speed));
        this.values.put(StepValues.distance.toString(),  (_distance));
        this.values.put(StepValues.parallel.toString(),  (_parallel));
        for (int i = 0; i < _parms.length; i++) {
            this.values.put(StepValues.parm.toString() + i,  _parms[i]);   
        }
    }
    public Step () {

    }

    public void setValue(StepValues key, Object val) {
        this.values.replace(key.toString(), String.valueOf(val));
    }

    public String getCommand() {
        return this.values.get(StepValues.command.toString());
    }

    public double getSpeed() {
        return Double.parseDouble(this.values.get(StepValues.speed.toString()));
    }

    public double getDistance() {
        return Double.parseDouble(this.values.get(StepValues.distance.toString()));
    }

    public boolean isParallel() {
        return Boolean.parseBoolean(this.values.get(StepValues.parallel.toString()));
    }

    public void addParm(Double value) {
        
    }

    public Double getParm(int parm) {
        if (this.values.containsKey(StepValues.parm.toString() + parm)) {
            return Double.parseDouble(this.values.get(StepValues.parm.toString() + parm));
        } else {
            return Double.NaN;
        }
    }

    public Double getParm(int parm, Double falseReturn) {
        if (this.values.containsKey(StepValues.parm.toString() + parm)) {
            return Double.parseDouble(this.values.get(StepValues.parm.toString() + parm));
        } else {
            return falseReturn;
        }
    }

    public Integer getParmInt(int parm) {
        if (this.values.containsKey(StepValues.parm.toString() + parm)) {
            return Integer.valueOf(this.values.get(StepValues.parm.toString() + parm));
        } else {
            return null;
        }
    }

    public boolean checkParm(int parm) {
        return this.values.containsKey(StepValues.parm.toString() + parm);
    }
}