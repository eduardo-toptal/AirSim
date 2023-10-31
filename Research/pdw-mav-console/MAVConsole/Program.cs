using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using static MAVLink;

#pragma warning disable CS8600 
#pragma warning disable CS8604

namespace MyApp // Note: actual namespace depends on the project name.
{
    internal class Program {

        static byte[] buffer = new byte[512];

        static string tcp_host_ip = "172.27.80.1";
        static int    tcp_port    = 4560;

        static string udp_ip_local  = tcp_host_ip;
        static string udp_ip_remote = "172.27.87.65";
        static string udp_gcs_ip    = "127.0.0.1";

        static int udp_port_local   = 14540;
        static int udp_port_gcs     = 14550;
        static int udp_port_remote  = 14580;

        static byte target_sys = 1;

        static byte target_comp = 1;


        static bool has_init = false;

        static float Noise(float x,float s) {
            float n = (float)((rnd.NextDouble() - 0.5) * 2.0);
            return x + (n*s);
        }
        static Random rnd = new Random();

        static void Main(string[] args) {

            TcpListener srv = new TcpListener(IPAddress.Parse(tcp_host_ip),tcp_port);
            srv.Start();

            Task<TcpClient> tsk = srv.AcceptTcpClientAsync();
            TcpClient     tcp_cl;
            NetworkStream tcp_s;
            UdpClient?    udp_remote;
            UdpClient?    udp_gcs;
            
            while (true) {
                switch(tsk.Status) {

                    default: {
                        Thread.Sleep(800);
                        Console.WriteLine($"MAVLink> Waiting TCP / addr: {tcp_host_ip}:{tcp_port}");
                    }
                    break;

                    case TaskStatus.RanToCompletion: {
                        if (has_init) break;
                        has_init = true;
                        tcp_cl   = tsk.Result;
                        tcp_s    = tcp_cl.GetStream();
                        udp_remote   = null;
                        udp_gcs      = null;

                        IPEndPoint remote_ep  = tcp_cl.Client.RemoteEndPoint == null ? null : (IPEndPoint)tcp_cl.Client.RemoteEndPoint;
                        IPEndPoint local_ep   = tcp_cl.Client.LocalEndPoint  == null ? null : (IPEndPoint)tcp_cl.Client.LocalEndPoint;
                        string remote_addr = remote_ep == null ? "<null>" : remote_ep.Address.ToString();
                        string remote_port = remote_ep == null ? "<null>" : remote_ep.Port.ToString();
                        string local_addr  = local_ep  == null ? "<null>" : local_ep.Address.ToString();
                        string local_port  = local_ep  == null ? "<null>" : local_ep.Port.ToString();

                        Console.WriteLine($"MAVLink> Connection Complete / success: {tcp_cl.Connected}");
                        Console.WriteLine($"  REMOTE: {remote_addr}:{remote_port}");
                        Console.WriteLine($"  LOCAL:  {local_addr}:{local_port}");

                        MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse(true);

                        MAVLink.mavlink_heartbeat_t heart_beat_d = default;
                        bool has_heart_beat = false;

                        Stopwatch clk = new Stopwatch();
                        clk.Start();
                        
                        Queue<MAVLink.MAVLinkMessage> rcv_queue = new Queue<MAVLink.MAVLinkMessage>();
                        Queue<MAVLink.MAVLinkMessage> snd_queue = new Queue<MAVLink.MAVLinkMessage>();

                        Thread thd_receive_tcp = 
                        new Thread(delegate() {
                            Console.WriteLine($"MAVLink> [TCP] [R] Start");
                            while (true) {
                                //CPU Yield
                                Thread.Sleep(10);
                                if (!tcp_cl.Connected) { Console.WriteLine($"MAVLink> [TCP] [R] Thread Stop!"); break; }                                
                                //Wait for Messages
                                //Console.WriteLine($"MAVLink> [TCP] [R] Poll");
                                MAVLink.MAVLinkMessage msg = MAVLink.MAVLinkMessage.Invalid;                                
                                try {

                                    if (tcp_s.CanRead) {                                        
                                        msg = parser.ReadPacket(tcp_s);
                                    }
                                }
                                catch(System.Exception p_err) {
                                    Console.WriteLine($"MAVLink> [TCP] [R] Error: {p_err.Message}");
                                }                                
                                if(msg!=null) if(msg.msgid != (uint)MAVLink.MAVLINK_MSG_ID.HIL_ACTUATOR_CONTROLS) Console.WriteLine($"MAVLink> [TCP] [R] RCV: {msg} | {tcp_s}");
                                lock(rcv_queue) {
                                    rcv_queue.Enqueue(msg);
                                }                                
                            }
                        });
                        thd_receive_tcp.Start();

                        Thread thd_receive_udp =
                        new Thread(delegate () {
                            Console.WriteLine($"MAVLink> [UDP] [R] Start");
                            IPEndPoint udp_gcs_ep = new IPEndPoint(IPAddress.Parse(udp_gcs_ip),udp_port_gcs);
                            MemoryStream ms = new MemoryStream(2048);
                            MemoryStream cp = new MemoryStream(2048);
                            while (true) {
                                //CPU Yield
                                Thread.Sleep(10);
                                if (!tcp_cl.Connected) { Console.WriteLine($"MAVLink> [UDP] [H] Thread Stop!"); break; }
                                //Console.WriteLine($"MAVLink> [UDP] [R] Poll");
                                MAVLink.MAVLinkMessage msg = MAVLink.MAVLinkMessage.Invalid;
                                long ms_pos = ms.Position;
                                try {
                                    //Wait for Messages
                                    int len = udp_gcs == null ? -1 : udp_gcs.Available;
                                    byte[]? b = udp_gcs == null ? null : udp_gcs.Receive(ref udp_gcs_ep);
                                    int count = b == null ? -1 : b.Length;
                                    if (count <= 0) continue;                                    
                                    ms.Write(b,0,count);
                                    if (ms.Length < MAVLink.MAVLINK_MAX_PACKET_LEN) continue;
                                    ms.Position = 0;
                                    msg = parser.ReadPacket(ms);
                                    cp.Position = 0;
                                    cp.SetLength(0);
                                    ms.CopyTo(cp);
                                    cp.Position = 0;
                                    ms.Position = 0;
                                    ms.SetLength(0);
                                    cp.CopyTo(ms);
                                } 
                                catch (System.Exception p_err) {
                                    Console.WriteLine($"MAVLink> [UDP] [R] Error: {p_err.Message}");
                                    ms.Position = ms_pos;
                                    continue;
                                }
                                
                                if (msg != null) if (msg.msgid != (uint)MAVLink.MAVLINK_MSG_ID.HIL_ACTUATOR_CONTROLS) Console.WriteLine($"MAVLink> [UDP] [R] RCV: {msg} | {ms_pos} bytes");
                                lock (rcv_queue) {
                                    rcv_queue.Enqueue(msg);
                                }
                                //MAVLink.MAVLinkMessage msg = MAVLink.MAVLinkMessage.Invalid;                                
                                //Console.WriteLine($"MAVLink> [UDP] [R] RCV: {msg}");
                                //*/
                            }
                        });
                        thd_receive_udp.Start();


                        Thread thd_heartbeat =
                        new Thread(
                        delegate () {
                            Console.WriteLine($"MAVLink> [H] Start");
                            while (true) {                                
                                if (!tcp_cl.Connected) { Console.WriteLine($"MAVLink> [TCP] [H] Thread Stop!"); break; }
                                if (!has_heart_beat) continue;
                                lock(parser) {                                    
                                    byte[] d_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT,heart_beat_d,true,target_sys,target_comp);
                                    tcp_s.Write(d_buff);
                                    //udp_remote?.Send(d_buff,d_buff.Length);
                                    udp_gcs?.Send(d_buff,d_buff.Length);
                                    Console.WriteLine($"MAVLink> [TCP] [H] ---B---");
                                }
                                Thread.Sleep(900);
                            }
                        });
                        thd_heartbeat.Start();

                        Thread thd_ping =
                        new Thread(
                        delegate () {
                            Console.WriteLine($"MAVLink> [P] Start");
                            uint seq = 0;
                            while (true) {
                                if (!tcp_cl.Connected) { Console.WriteLine($"MAVLink> [TCP] [P] Thread Stop!"); break; }
                                if (!has_heart_beat) continue;
                                double us = ((double)clk.ElapsedTicks / (double)Stopwatch.Frequency) * 1000000;
                                lock (parser) {
                                    MAVLink.mavlink_ping_t ping_d = new mavlink_ping_t() {
                                        time_usec        = (ulong)us,
                                        seq              = seq++,
                                        target_system    = 1,
                                        target_component = 1
                                    };
                                    byte[] d_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.PING,ping_d,true,target_sys,target_comp);
                                    tcp_s.Write(d_buff);
                                    //udp_remote?.Send(d_buff,d_buff.Length);
                                    udp_gcs?.Send(d_buff,d_buff.Length);
                                    Console.WriteLine($"MAVLink> [ALL] [P] ---P---");
                                }
                                Thread.Sleep(900);
                            }
                        });
                        thd_ping.Start();


                        Thread thd_mavlink = new Thread(
                        delegate () {
                            Console.WriteLine($"MAVLink> [Q] Start");

                            int gps_slice = 0;

                            while (true) {
                                Thread.Sleep(10);
                                if (!tcp_cl.Connected) { Console.WriteLine($"MAVLink> [Q] Not Connected"); break; }
                                MAVLink.MAVLinkMessage msg = MAVLink.MAVLinkMessage.Invalid;
                                bool has_msg = false;

                                lock (rcv_queue) {
                                    if (rcv_queue.Count > 0) { msg = rcv_queue.Dequeue(); has_msg = msg!=null; }
                                }

                                if (has_heart_beat) {

                                    double us = ((double)clk.ElapsedTicks / (double)Stopwatch.Frequency)*1000000;
                                    byte[] msg_buff;

                                    #region SYS_STATUS
                                    MAVLink.mavlink_sys_status_t mavlink_sys_status_d = new mavlink_sys_status_t() {                                        
                                        battery_remaining = 100,
                                        onboard_control_sensors_enabled  = (((uint)1<<31)-1),
                                        onboard_control_sensors_health   = (((uint)1 << 31) - 1),
                                        onboard_control_sensors_present  = (((uint)1 << 31) - 1),
                                        current_battery = -100,
                                        drop_rate_comm = 0,
                                        load = 360,
                                        voltage_battery = 16200,                                        
                                        errors_comm =0,
                                        errors_count1 = 0,
                                        errors_count2 = 0,
                                        errors_count3 = 0,
                                        errors_count4 = 0
                                    };
                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.SYS_STATUS,mavlink_sys_status_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(hil_buff,hil_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    #endregion

                                    #region HIL_SENSOR
                                    
                                    MAVLink.mavlink_hil_sensor_t mavlink_hil_sensor_d = new MAVLink.mavlink_hil_sensor_t() {
                                        time_usec = (ulong)us,
                                        xacc  = Noise(   0f,0.1f),
                                        yacc  = Noise(   0f,0.1f),
                                        zacc  = Noise(-9.8f,0.1f),                                        
                                        xgyro = Noise(0f,0.01f),
                                        ygyro = Noise(0f,0.01f),
                                        zgyro = Noise(0f,0.01f),                                        
                                        xmag  = Noise( 0.20f,0.1f),
                                        ymag  = Noise(-0.70f,0.1f),
                                        zmag  = Noise( 0.45f,0.1f),
                                        temperature    = Noise(30f,0.1f),
                                        abs_pressure   = Noise(0f,0.1f),
                                        diff_pressure  = Noise(0,0.1f),
                                        pressure_alt   = Noise(50f,1f),
                                        fields_updated = ((uint)((uint)0xfff))
                                    };

                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HIL_SENSOR,mavlink_hil_sensor_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);                                        
                                        //udp_cl?.Send(hil_buff,hil_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region HIL_STATE
                                    
                                    MAVLink.mavlink_hil_state_t mavlink_hil_state_d = new MAVLink.mavlink_hil_state_t() {
                                        time_usec = (ulong)us,
                                        xacc  = (short)Noise(   0f,0.1f),
                                        yacc  = (short)Noise(   0f,0.1f),
                                        zacc  = (short)Noise(-9.8f,0.1f),
                                        alt = (int)(Noise(122.0f,1) * 1E3),
                                        lat = (int)(Noise(47.642406f,0.000001f) * 1E7),
                                        lon = (int)(Noise(-122.140977f,0.000001f) * 1E7),
                                        roll  = Noise(0f,0.01f),                                        
                                        pitch = Noise(0f,0.01f),
                                        yaw   = Noise(0f,0.01f),
                                        rollspeed  = Noise(0f,0.01f),
                                        pitchspeed = Noise(0f,0.01f),
                                        yawspeed   = Noise(0f,0.01f),
                                        vx       = (short)Noise(0f,1f),
                                        vy       = (short)Noise(0f,1f),
                                        vz       = (short)Noise(0f,1f)                                        
                                    };

                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HIL_STATE,mavlink_hil_state_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(hil_buff,hil_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region HIL_RC_INPUTS_RAW
                                    
                                    MAVLink.mavlink_hil_rc_inputs_raw_t mavlink_rc_raw_d = new MAVLink.mavlink_hil_rc_inputs_raw_t() {
                                        time_usec = (ulong)us,
                                        rssi      = (byte)Noise(0,50),
                                        chan1_raw = (ushort)Noise(1000f,0.0f),
                                        chan2_raw = (ushort)Noise(1000f,0.0f),
                                        chan3_raw = (ushort)Noise(1000f,0.0f),
                                        chan4_raw = (ushort)Noise(1000f,0.0f)
                                    };

                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HIL_RC_INPUTS_RAW,mavlink_rc_raw_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(hil_buff,hil_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region SET_ATTITUDE_TARGET
                                    /*
                                    MAVLink.mavlink_attitude_target_t mavlink_att_d = new MAVLink.mavlink_attitude_target_t() {
                                        time_boot_ms = (uint)(us / 1000),
                                        type_mask = 0,
                                        body_pitch_rate = Noise(0f,0f),
                                        body_yaw_rate   = Noise(0f,0f),
                                        body_roll_rate  = Noise(0f,0f),
                                        thrust          = Noise(0f,0f),
                                        q               = new float[] {1f,0f,0f,0f}
                                    };

                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.SET_ATTITUDE_TARGET,mavlink_att_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(msg_buff,msg_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region ATTITUDE      
                                    
                                    MAVLink.mavlink_attitude_t mavlink_att_t_d = new MAVLink.mavlink_attitude_t() {
                                        time_boot_ms = (uint)(us / 1000),                                        
                                        roll  = Noise(0f,0.01f),                                        
                                        pitch = Noise(0f,0.01f),
                                        yaw   = Noise(0f,0.01f),
                                        rollspeed  = Noise(0f,0.01f),
                                        pitchspeed = Noise(0f,0.01f),
                                        yawspeed   = Noise(0f,0.01f)                                        
                                    };
                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.ATTITUDE,mavlink_att_t_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(msg_buff,msg_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region ATTITUDE_QUATERNION  
                                    
                                    MAVLink.mavlink_attitude_quaternion_t mavlink_att_qt_d = new MAVLink.mavlink_attitude_quaternion_t() {
                                        time_boot_ms = (uint)(us / 1000),                                        
                                        q1 = Noise(1f,0.01f),
                                        q2 = Noise(0f,0.01f),
                                        q3 = Noise(0f,0.01f),
                                        q4 = Noise(0f,0.01f),
                                        rollspeed  = Noise(0f,0.01f),
                                        pitchspeed = Noise(0f,0.01f),
                                        yawspeed   = Noise(0f,0.01f)
                                    };
                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.ATTITUDE_QUATERNION,mavlink_att_qt_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(msg_buff,msg_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region ATTITUDE_TARGET
                                    
                                    MAVLink.mavlink_attitude_target_t mavlink_att_tgt_d = new MAVLink.mavlink_attitude_target_t() {
                                        time_boot_ms = (uint)(us / 1000),
                                        q  = new float[] { Noise(1f,0.01f), Noise(0f,0.01f),Noise(0f,0.01f),Noise(0f,0.01f) },
                                        body_yaw_rate   = Noise(0f,0.01f),
                                        body_pitch_rate = Noise(0f,0.01f),
                                        body_roll_rate  = Noise(0f,0.01f),
                                        thrust          = Noise(0f,0.01f),
                                        type_mask       = 0
                                    };
                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.ATTITUDE_TARGET,mavlink_att_tgt_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(msg_buff,msg_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region HIGHRES_IMU
                                    
                                    MAVLink.mavlink_highres_imu_t mavlink_highres_imu_d = new MAVLink.mavlink_highres_imu_t() {
                                        time_usec = (ulong)us,
                                        xacc = Noise(0f,0.1f),
                                        yacc = Noise(0f,0.1f),
                                        zacc = Noise(-9.8f,0.1f),
                                        xgyro = Noise(0f,0.01f),
                                        ygyro = Noise(0f,0.01f),
                                        zgyro = Noise(0f,0.01f),
                                        xmag = Noise(0.20f,0.1f),
                                        ymag = Noise(-0.70f,0.1f),
                                        zmag = Noise(0.45f,0.1f),
                                        temperature = Noise(30f,0.1f),
                                        abs_pressure = Noise(0f,0.1f),
                                        diff_pressure = Noise(0,0.1f),
                                        pressure_alt = Noise(50f,1f),
                                        fields_updated = ((ushort)((ushort)0xfff))
                                    };
                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HIGHRES_IMU,mavlink_highres_imu_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(msg_buff,msg_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region TIME_SYNC
                                    
                                    MAVLink.mavlink_timesync_t mavlink_timesync_d = new MAVLink.mavlink_timesync_t() {
                                        tc1 = 0,
                                        ts1 = (long)us
                                    };
                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.TIMESYNC,mavlink_timesync_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(msg_buff,msg_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region ALTITUDE
                                    
                                    MAVLink.mavlink_altitude_t mavlink_altitude_d = new MAVLink.mavlink_altitude_t() {
                                        time_usec = (ulong)us,
                                        altitude_amsl       = Noise(0.1f,0.1f),
                                        altitude_local      = Noise(0.1f,0.1f),
                                        altitude_monotonic  = Noise(0.1f,0.1f),
                                        altitude_relative   = Noise(0.1f,0.1f),
                                        altitude_terrain    = Noise(0.1f,0.1f),
                                        bottom_clearance    = Noise(0.1f,0.1f)
                                    };
                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.ALTITUDE,mavlink_altitude_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(msg_buff,msg_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    #region VFR_HUD
                                    
                                    MAVLink.mavlink_vfr_hud_t mavlink_vfr_hud_d = new MAVLink.mavlink_vfr_hud_t() {
                                        airspeed    = Noise(0.1f,0.1f),
                                        alt         = Noise(0.1f,0.1f),
                                        climb       = Noise(0.1f,0.1f),
                                        groundspeed = Noise(0.1f,0.1f),
                                        heading     = (short)Noise(45f,45f),
                                        throttle    = (ushort)Noise(0.1f,0.1f)
                                    };
                                    msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.VFR_HUD,mavlink_vfr_hud_d,true,target_sys,target_comp);
                                    try {
                                        if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                        //udp_cl?.Send(msg_buff,msg_buff.Length);
                                    } catch (System.Exception p_err) {
                                        Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                    }
                                    //*/
                                    #endregion

                                    gps_slice++;
                                    if (gps_slice == 10) {
                                        gps_slice  = 0;

                                        #region HIL_GPS
                                        
                                        MAVLink.mavlink_hil_gps_t mavlink_gps_d = new MAVLink.mavlink_hil_gps_t() {
                                            time_usec = (ulong)us,                                            
                                            alt = (int)(Noise(122.0f,1) * 1E3),
                                            lat = (int)(Noise(47.642406f,0.000001f) * 1E7),
                                            lon = (int)(Noise(-122.140977f,0.000001f) * 1E7),
                                            eph = (ushort)(Noise(1,0.1f) * 100),
                                            epv = (ushort)(Noise(1,0.1f) * 100),
                                            fix_type = 3,
                                            satellites_visible = 10,
                                            vd  = (short )(Noise(0,0.1f) * 100), // cm/s
                                            ve  = (short )(Noise(0,0.1f) * 100), // cm/s
                                            vn  = (short )(Noise(0,0.1f) * 100), // cm/s
                                            vel = (ushort)(Math.Abs(Noise(0,0.1f) * 100)), // cm/s
                                            cog = (ushort)(Noise(0,0.3f) * 100), // degrees * 100
                                        };

                                        msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HIL_GPS,mavlink_hil_sensor_d,true,target_sys,target_comp);
                                        try {
                                            if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                            //udp_cl?.Send(hil_buff,hil_buff.Length);
                                        } catch (System.Exception p_err) {
                                            Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                        }
                                        //*/
                                        #endregion

                                        #region GPS_RAW_INT
                                        MAVLink.mavlink_gps_raw_int_t mavlink_gps_raw_d = new MAVLink.mavlink_gps_raw_int_t() {
                                            time_usec = (ulong)us,
                                            alt = (int)(Noise(122.0f,1) * 1E3),
                                            lat = (int)(Noise(47.642406f,0.000001f) * 1E7),
                                            lon = (int)(Noise(-122.140977f,0.000001f) * 1E7),
                                            eph = (ushort)(Noise(0,0.1f) * 100),
                                            epv = (ushort)(Noise(0,0.1f) * 100),
                                            cog = (ushort)(Noise(0,0.3f) * 100), // degrees * 100
                                            vel = (ushort)(Noise(0,0.3f)), // cm/s
                                            fix_type = 0,
                                            satellites_visible = 10,
                                            alt_ellipsoid = (int)Noise(0f,0.01f),
                                            h_acc   = (uint)Noise(0f,0.01f),
                                            v_acc   = (uint)Noise(0f,0.01f),
                                            vel_acc = (uint)(Noise(0,0.1f)),
                                            hdg_acc = (uint)(Noise(0,0.1f))                                            
                                        };

                                        msg_buff = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT,mavlink_gps_raw_d,true,target_sys,target_comp);
                                        try {
                                            if (tcp_s.CanWrite) tcp_s.Write(msg_buff);
                                            //udp_cl?.Send(msg_buff,msg_buff.Length);
                                        } catch (System.Exception p_err) {
                                            Console.WriteLine($"MAVLink> [TCP] [Q] Error: {p_err.Message}");
                                        }
                                        #endregion

                                    }

                                }

                                if (!has_msg) continue;

                                switch(msg?.msgtypename) {
                                    case "HEARTBEAT": {

                                        if (has_heart_beat) break;

                                        MAVLink.mavlink_heartbeat_t hb = (MAVLink.mavlink_heartbeat_t)msg.data;
                                        Console.WriteLine($"MAVLink> [Q] HEARTBEAT Received: {msg}");
                                        Console.WriteLine($"  MAV_AUTOPILOT:   {(MAVLink.MAV_AUTOPILOT)hb.autopilot}");
                                        Console.WriteLine($"  MAVLINK_VERSION: {hb.mavlink_version}");
                                        Console.WriteLine($"  MAV_TYPE:        {(MAVLink.MAV_TYPE)hb.type}");
                                        Console.WriteLine($"  BASEMODE:        {hb.base_mode}");
                                        Console.WriteLine($"  MAV_STATE:       {(MAVLink.MAV_STATE)hb.system_status}");

                                        if(hb.type == (byte)MAVLink.MAV_TYPE.GCS) {
                                            break;
                                        }

                                        hb.system_status = (byte)MAVLink.MAV_STATE.STANDBY;                                        
                                        hb.type      = (byte)MAVLink.MAV_TYPE.QUADROTOR;
                                        hb.base_mode = (byte)(MAVLink.MAV_MODE_FLAG.HIL_ENABLED | MAVLink.MAV_MODE_FLAG.STABILIZE_ENABLED);
                                        heart_beat_d     = hb;

                                        //Thread.Sleep(300);

                                        Console.WriteLine($"MAVLink> Creating UDP Client");
                                        Console.WriteLine($"  local :  {udp_ip_local}:{udp_port_local}");
                                        Console.WriteLine($"  remote: {udp_ip_remote}:{udp_port_remote}");
                                        Console.WriteLine($"  GCS   :   {udp_gcs_ip}:{udp_port_gcs}");
                                        try {

                                            IPEndPoint udp_local_ep  = new IPEndPoint(IPAddress.Parse(udp_ip_local ),udp_port_local);
                                            IPEndPoint udp_remote_ep = new IPEndPoint(IPAddress.Parse(udp_ip_remote),udp_port_remote);
                                            IPEndPoint udp_gcs_ep    = new IPEndPoint(IPAddress.Parse(udp_gcs_ip),udp_port_gcs);

                                            udp_remote = new UdpClient();                                            
                                            udp_remote.Connect(udp_remote_ep);

                                            udp_gcs = new UdpClient();
                                            udp_gcs.Connect(udp_gcs_ep);

                                            Console.WriteLine($"MAVLink> UDP Connected: {udp_remote.Client.Connected}");
                                            
                                        } catch (System.Exception p_err) {
                                            Console.WriteLine($"MAVLink> UDP Client Error / Error: {p_err.Message}");                                            
                                        }

                                        has_heart_beat = true;

                                        /*
                                        Console.WriteLine($"MAVLink> Sending DO_SET_MODE");
                                        
                                        MAVLink.mavlink_command_long_t mav_cmd_l = new MAVLink.mavlink_command_long_t() {
                                            command          = (ushort)MAVLink.MAV_CMD.DO_SET_MODE,
                                            param1           = (float)(MAVLink.MAV_MODE_FLAG.HIL_ENABLED | MAVLink.MAV_MODE_FLAG.SAFETY_ARMED | MAVLink.MAV_MODE_FLAG.GUIDED_ENABLED),
                                            confirmation     = 0,
                                            target_component = 0
                                        };

                                        byte[] msg_b;

                                        msg_b = parser.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,mav_cmd_l,true,target_sys,target_comp);
                                        tcp_s.Write(msg_b);
                                        //*/
                                        
                                    }
                                    break;

                                    default: {
                                        //Console.WriteLine($"MavLink> [Q] {msg?.msgtypename} Received!!!!");
                                    }
                                    break;

                                }

                            }                            
                        });
                        thd_mavlink.Start();

                    }
                    break;

                }
                    
            }
        }
    }
}
/*
INFO  [commander] Preflight check: FAILED
WARN  [PreFlightCheck] Arming denied! angular velocity invalid
WARN  [PreFlightCheck] Arming denied! attitude invalid
WARN  [PreFlightCheck] Arming denied! manual control lost
INFO  [commander] Prearm check: FAILED
INFO  [HealthFlags] DEVICE              STATUS
INFO  [HealthFlags] ----------------------------------
INFO  [HealthFlags] GYRO:
INFO  [HealthFlags] ACC:
INFO  [HealthFlags] MAG:                EN      OFF
INFO  [HealthFlags] PRESS:              EN      OFF
INFO  [HealthFlags] AIRSP:
INFO  [HealthFlags] GPS:
INFO  [HealthFlags] OPT:
INFO  [HealthFlags] VIO:
INFO  [HealthFlags] LASER:
INFO  [HealthFlags] GTRUTH:
INFO  [HealthFlags] RATES:
INFO  [HealthFlags] ATT:
INFO  [HealthFlags] YAW:
INFO  [HealthFlags] ALTCTL:
INFO  [HealthFlags] POS:
INFO  [HealthFlags] MOT:                EN      OK
INFO  [HealthFlags] RC  :
INFO  [HealthFlags] GYRO2:
INFO  [HealthFlags] ACC2:
INFO  [HealthFlags] MAG2:
INFO  [HealthFlags] GEOFENCE:
INFO  [HealthFlags] AHRS:                       ERR
INFO  [HealthFlags] TERRAIN:
INFO  [HealthFlags] REVMOT:
INFO  [HealthFlags] LOGGIN:
INFO  [HealthFlags] BATT:
INFO  [HealthFlags] PROX:
INFO  [HealthFlags] SATCOM:
INFO  [HealthFlags] PREARM:
INFO  [HealthFlags] OBSAVD:                     ERR

WARN  [PreFlightCheck] Preflight: GPS Vertical Pos Drift too high
INFO  [commander] Preflight check: OK
INFO  [commander] Prearm check: OK
INFO  [HealthFlags] DEVICE              STATUS
INFO  [HealthFlags] ----------------------------------
INFO  [HealthFlags] GYRO:
INFO  [HealthFlags] ACC:
INFO  [HealthFlags] MAG:                EN      OK
INFO  [HealthFlags] PRESS:              EN      OK
INFO  [HealthFlags] AIRSP:
INFO  [HealthFlags] GPS:                        ERR
INFO  [HealthFlags] OPT:
INFO  [HealthFlags] VIO:
INFO  [HealthFlags] LASER:
INFO  [HealthFlags] GTRUTH:
INFO  [HealthFlags] RATES:
INFO  [HealthFlags] ATT:
INFO  [HealthFlags] YAW:
INFO  [HealthFlags] ALTCTL:
INFO  [HealthFlags] POS:
INFO  [HealthFlags] MOT:                EN      OK
INFO  [HealthFlags] RC  :               EN      OK
INFO  [HealthFlags] GYRO2:
INFO  [HealthFlags] ACC2:
INFO  [HealthFlags] MAG2:               EN      OK
INFO  [HealthFlags] GEOFENCE:
INFO  [HealthFlags] AHRS:               EN      OK
INFO  [HealthFlags] TERRAIN:
INFO  [HealthFlags] REVMOT:
INFO  [HealthFlags] LOGGIN:
INFO  [HealthFlags] BATT:
INFO  [HealthFlags] PROX:
INFO  [HealthFlags] SATCOM:
INFO  [HealthFlags] PREARM:             EN      OK
INFO  [HealthFlags] OBSAVD:

//*/