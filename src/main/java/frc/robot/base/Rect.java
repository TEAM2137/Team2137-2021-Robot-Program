package frc.robot.base;

public class Rect {
    public double x, y;
    public double height, width;

    public Rect (double _x, double _y, double _height, double _width) {
        this.x = _x;
        this.y = _y;
        this.height = _height;
        this.width = _width;
    }

    public double[][] getPointData() {
        return new double[][] {
                {x, y},
                {x + width, y},
                {x + width, y + height},
                {x, y + height}
        };
    }
}
