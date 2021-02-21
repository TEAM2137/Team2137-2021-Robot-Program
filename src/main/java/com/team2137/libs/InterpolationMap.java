package com.team2137.libs;

import java.util.TreeMap;

public class InterpolationMap extends TreeMap<Double, Double> {

    public Double getInterpolated(Double key) {
        Double value = get(key);
        if(value == null) {
            Double topKey = ceilingKey(key);
            Double bottomKey = ceilingKey(key);

            // if there isn't a value above or below, return the other
            // if neither, 0
            if(topKey == null && bottomKey == null) {
                return 0.0;
            } else if (topKey == null) {
                return get(bottomKey);
            } else if (bottomKey == null) {
                return get(topKey);
            }

            Double topValue = get(topKey);
            Double bottomValue = get(bottomKey);

            return (topValue * (bottomKey - key) + bottomValue * (key - topKey)) / (bottomKey - topKey); // actually interpolate
        } else {
            return value;
        }
    }
}
