package com.team2137.frc2021.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LimeLight extends SubsystemBase {
    public enum LimeLightValues {
        TX ("tx"),
        TY ("ty"),
        TA ("ta"),
        TV ("tv"),
        LEDMODE ("ledMode"),
        CAMMODE ("camMode"),
        PIPELINE ("pipeLine"),;

        public String tableName = "";

        LimeLightValues (String tableValue) {
            tableName = tableValue;
        }

        public String getTableName() {
            return tableName;
        }

        public double getValue(NetworkTable table) {
            return table.getEntry(tableName).getDouble(0.0);
        }
    }

    private double tx, ty, ta;
    private boolean tv;
    private NetworkTable networkTable;

    public LimeLight(NetworkTable _table) {
        networkTable = _table;

        networkTable.addEntryListener(LimeLightValues.TX.getTableName(), (table, key, entry, value, flags) -> {
            tx = Math.toRadians(value.getDouble());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        networkTable.addEntryListener(LimeLightValues.TY.getTableName(), (table, key, entry, value, flags) -> {
            ty = Math.toRadians(value.getDouble());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        networkTable.addEntryListener(LimeLightValues.TA.getTableName(), (table, key, entry, value, flags) -> {
            ta = value.getDouble();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        networkTable.addEntryListener(LimeLightValues.TV.getTableName(), (table, key, entry, value, flags) -> {
            tv = value.getDouble() >= 1;
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

//        tx = Math.toRadians(LimeLightValues.TX.getValue(_table));
//        ty = Math.toRadians(LimeLightValues.TY.getValue(_table));
//        tv = LimeLightValues.TV.getValue(_table) == 1;
//        ta = Math.toRadians(LimeLightValues.TX.getValue(_table));
    }

    @Override
    public abstract void periodic();

    public double getLimeLightValue(LimeLightValues values) {
        switch(values) {
            case TX:
                return tx;
            case TY:
                return ty;
            case TA:
                return ta;
            case TV:
                return tv ? 1 : -1;
            default:
                return 0;
        }
    }

    public boolean hasValidTarget() {
        return tv;
    }

    public void enableLED() {
        networkTable.getEntry(LimeLightValues.LEDMODE.getTableName()).setNumber(3);
    }

    public void disableLED() {
        networkTable.getEntry(LimeLightValues.LEDMODE.getTableName()).setNumber(1);
    }
}
