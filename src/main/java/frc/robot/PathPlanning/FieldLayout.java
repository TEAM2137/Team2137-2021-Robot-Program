package frc.robot.PathPlanning;

import frc.robot.base.Rect;

import java.awt.*;

public class FieldLayout {
    public enum InfiniteRechargeFieldLayout {
        FIELD (new Rect(0, 0, 26.9375, 26.4375)),
        INITIATION_LINE(new Rect(10, 0, 26.9375, 2)),
        GOAL (new Rect(0,0,2, 3));

        private Rect element;

        InfiniteRechargeFieldLayout(Rect rect) {
            element = rect;
        }

        public Rect getFieldElement() {
            return this.element;
        }
    }

    public FieldLayout () {
        //Lets create a bank image
        LinePlot fig3 = new LinePlot(new double[][]{{0.0,0.0}});
        fig3.yGridOn();
        fig3.xGridOn();
        fig3.setYLabel("Y (feet)");
        fig3.setXLabel("X (feet)");
        fig3.setTitle("Top Down View of FRC Field (52' 5.25\" x 26' 11.25\") \n shows global position of robot path, along with left and right wheel trajectories");

        //fig3.addData();

        //force graph to show 1/2 field dimensions of 24.8ft x 27 feet
        double fieldWidth = 32.0;
        fig3.setXTic(0, 27, 1);
        fig3.setYTic(0, fieldWidth, 1);


        //lets add field markers to help visual
        //http://www.usfirst.org/sites/default/files/uploadedFiles/Robotics_Programs/FRC/Game_and_Season__Info/2014/fe-00037_RevB.pdf
        //Goal line
        double[][] goalLine = new double[][] {{26.5,0}, {26.5, fieldWidth}};
        fig3.addData(goalLine, Color.black);

        //Low Goals roughly 33 inch x 33 inch and 24.6 ft apart (inside to inside)
        double[][] leftLowGoal = new double[][]{
                {26.5, fieldWidth/2 + 24.6/2},
                {26.5, (fieldWidth)/2 + 24.6/2 + 2.75},
                {26.5 - 2.75, fieldWidth/2 + 24.6/2 + 2.75},
                {26.5 - 2.75, fieldWidth/2 + 24.6/2},
                {26.5, fieldWidth/2 + 24.6/2},
        };

        double[][] rightLowGoal = new double[][]{
                {26.5, fieldWidth/2 - 24.6/2},
                {26.5, fieldWidth/2 - 24.6/2 - 2.75},
                {26.5 - 2.75, fieldWidth/2 - 24.6/2 - 2.75},
                {26.5 - 2.75, fieldWidth/2 - 24.6/2},
                {26.5, fieldWidth/2 - 24.6/2},
        };

        fig3.addData(leftLowGoal, Color.black);
        fig3.addData(rightLowGoal, Color.black);

    }
}
