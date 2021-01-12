package frc.robot.LIDAR;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.opencv.core.Point;

import javax.annotation.Nullable;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.*;
import java.util.List;

public class Main {

    public class DistanceObject {
        Point rawPoint;
        Point point;

        DistanceObject(int _distance, int _angle) {
            this.rawPoint = new Point(_distance, _angle);
            this.point = new Point(Math.cos(Math.toRadians(_angle)) * _distance, Math.sin(Math.toRadians(_angle)) * _distance);
        }

        DistanceObject(@Nullable Integer _distance, @Nullable Integer _angle, @Nullable Integer _x, @Nullable Integer _y) {
            if(_distance != null && _angle != null)
                this.rawPoint = new Point(_distance, _angle);

            if(_x != null && _y != null)
                this.point = new Point(_x, _y);
        }

        public int getX() {
            return (int) this.point.x;
        }

        public int getY() {
            return (int) this.point.y;
        }

        public int getDistance() {
            return (int) this.rawPoint.x;
        }

        public int getAngle() {
            return (int) this.rawPoint.y;
        }
    }

    public class R2300Packet {
        int magic;
        int packet_type;
        long packet_size;
        int header_size;
        int scan_number;
        int packet_number;
        long timestamp_raw;
        long timestamp_sync;
        long status_flags;
        long scan_frequency;
        int num_points_scan;
        int num_points_packet;
        int first_index;
        int first_angle;
        int angular_increment;
        long iq_input;
        long iq_overload;
        long iq_timestamp_raw;
        long iq_timestamp_sync;

        R2300Packet(int _magic, int _packet_type, int _packet_size, int _header_size, int _scan_number, int _packet_number, int _timestamp_raw, int _timestamp_sync, int _status_flags, int _scan_frequency, int _num_points_scan, int _num_points_packet, int _first_index, int _angular_increment, int _iq_output, int _iq_overload, int _iq_timestmp_raw, int _iq_timestmp_sync) {
            this.magic = _magic;
            this.packet_type = _packet_type;
            this.packet_size = _packet_size;
            this.header_size = _header_size;
            this.scan_number = _scan_number;
            this.packet_number = _packet_number;
            this.timestamp_raw = _timestamp_raw;
        }
    }

    public enum R2300URLParameters {
        PARAMETERS(""),
        VENDOR(""),
        PRODUCT(""),
        PART(""),
        SERIAL(""),
        REVISION_FW(""),
        REVISION_HW(""),
        MAX_CONNECTIONS(""),
        FEATURE_FLAGS(""),
        RADIAL_RANGE_MIN(""),
        RADIAL_RANGE_MAX(""),
        ANGULAR_FOV(""),
        ANGULAR_RESOLUTION(""),
        IP_MODE(""),
        IP_ADDRESS(""),
        SUBNET_MASK(""),
        GATEWAY(""),
        sCAN_FREQUENCY(""),
        SCAN_DIRECTION(""),
        SAMPLES_PER_SCAN(""),
        SCAN_FREQUENCY_MEASURED(""),
        STATUS_FLAGS(""),
        LOAD_INDICATION(""),
        DEVICE_FAMILY(""),
        MAC_ADDRESS(""),
        HMI_DISPLAY_MODE(""),
        HMI_LANGUAGE(""),
        HMI_BUTTON_LOCK(""),
        HMI_PARAMETER_LOCK(""),
        IP_MODE_CURRENT(""),
        IP_ADDRESS_CURRENT(""),
        SUBNET_MASK_CURRENT(""),
        GATEWAY_CURRENT(""),
        SYSTEM_TIME_RAW(""),
        USER_TAG(""),
        USER_NOTES(""),
        LOCATOR_INDICATION(""),
        TEMPERATURE_CURRENT(""),
        TEMPERATURE_MIN(""),
        TEMPERATURE_MAX("");

        public String value = "";

        R2300URLParameters(String name) {
            this.value = name;
        }

        public String getString() {
            return this.value;
        }
    }

    public enum R2300URLCommands {
        REQUEST_HANDLE ("request_handle_udp"),
        START_SCAN ("start_scanoutput"),
        STOP_SCAN ("stop_scanoutput"),
        RELEASE_HANDLE ("release_handle"),
        GET_PARAMETER ("get_parameter"),
        SET_PARAMETER ("set_parameter"),
        RESET_PARAMETER ("reset_parameter"),
        SCAN_CONFIG ("get_scanoutput_config"),
        FACTORY_RESET ("factory_reset"),
        REBOOT_DEVICE ("reboot_device");

        private String name = "";

        R2300URLCommands (String str) {
            this.name = str;
        }

        public String getString() {
            return this.name;
        }

        public URL buildURL(String R2300IP, String clientIP, String port) {
            try {
                return new URL("http://" + R2300IP + "/cmd/" + this.name + "?address=" + clientIP + "&port=" + port);
            } catch (MalformedURLException e) {
                e.printStackTrace();
                return null;
            }
        }

        public URL buildURL(String R2300IP, R2300URLParameters parm) {
            try {
                return new URL("http://" + R2300IP + "/cmd/" + this.name + "?" + parm.getString());
            } catch (MalformedURLException e) {
                e.printStackTrace();
                return null;
            }
        }

        public URL buildURL(String R2300IP, R2300URLParameters parm, String value) {
            try {
                return new URL("http://" + R2300IP + "/cmd/" + this.name + "?" + parm.getString() + "=" + value);
            } catch (MalformedURLException e) {
                e.printStackTrace();
                return null;
            }
        }

        public URL buildURL(String R2300IP, String handle) {
            try {
                return new URL("http://" + R2300IP + "/cmd/" + this.name + "?handle=" + handle);
            } catch (MalformedURLException e) {
                e.printStackTrace();
                return null;
            }
        }
    }

    public class R2300 {
        private DatagramSocket datagramSocket;

        private String strR2300IP;
        private byte[] R2300IP;
        private String strClientIP;
        private byte[] ClientIP;
        private String strPort;
        private int Port;

        R2300(byte[] R2300IP, byte[] clientIP, int port) {
            this.strR2300IP = String.valueOf(R2300IP[0]) + "." + String.valueOf(R2300IP[1]) + "." + String.valueOf(R2300IP[2]) + "." + String.valueOf(R2300IP[3]);
            this.R2300IP = R2300IP;
            this.strClientIP = String.valueOf(clientIP[0]) + "." + String.valueOf(clientIP[1]) + "." + String.valueOf(clientIP[2]) + "." + String.valueOf(clientIP[3]);
            this.ClientIP = clientIP;
            this.strPort = String.valueOf(port);
            this.Port = port;
        }

        public JSONObject sendGETCommand(R2300URLParameters parm) throws IOException {
            JSONTokener tokener = new JSONTokener(R2300URLCommands.GET_PARAMETER.buildURL(this.strR2300IP, parm).openStream());
            return new JSONObject(tokener);
        }

        public void sendSETCommand(R2300URLParameters parm, String value) throws IOException {
            R2300URLCommands.SET_PARAMETER.buildURL(this.strR2300IP, parm, value).openStream();
        }

        public String requestHandle() throws IOException {
            JSONTokener tokener = new JSONTokener(R2300URLCommands.REQUEST_HANDLE.buildURL(this.strR2300IP, this.strClientIP, this.strPort).openStream());
            return (new JSONObject(tokener)).getString("handle");
        }

        public void setCommand(String Handle) throws IOException {
            R2300URLCommands.START_SCAN.buildURL(this.strR2300IP, Handle).openStream();
        }

        public DatagramSocket openUDPSocket() throws SocketException {
            this.datagramSocket = new DatagramSocket(this.Port);
            return this.datagramSocket;
        }

        public List<DistanceObject> receivePacket() {
            byte[] buf = new byte[2048];
            if(this.datagramSocket == null){
                try {
                    openUDPSocket();
                } catch (SocketException e) {
                    e.printStackTrace();
                }
            }

            DatagramPacket packet = new DatagramPacket(buf, buf.length);
            try {
                this.datagramSocket.receive(packet);
            } catch (IOException e) {
                e.printStackTrace();
                return null;
            }

            return null;
        }

        private int translateUnsigned16(byte[] data) {
            return (data[0]) + (data[1] << 8);
        }

        private int translateUnsigned32(byte[] data) {
            return (data[0]) + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
        }

        private int translateSigned32(byte[] data) {
            long tmp = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
            long sign = tmp >> 31;

            if(sign == 1){
                tmp = (~tmp) + 0b1;
                return (int) tmp * -1;
            } else {
                return (int) tmp;
            }
        }
    }

    public static void main(String[] args) throws IOException {
        Main m = new Main();
        m.test();
    }

    public void test() throws IOException {
        byte[] tmp = {10, 0, 10, 9};
        byte[] tmp2 = {10, 0, 10, 10};
        R2300 lidar = new R2300(tmp, tmp2, 4445);
        String handle = "";
        try {
            handle = lidar.requestHandle();
        } catch (IOException e) {
            e.printStackTrace();
        }
        lidar.setCommand(handle);

    }
}
