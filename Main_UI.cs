using System;
using System.Collections.Generic;
using System.Timers;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.Util;
using Emgu.CV.Features2D;
using TwinCAT.Ads;
using System.Diagnostics;
using System.Threading;
using System.IO;
using PylonC.NET;
using MySql.Data.MySqlClient;
using System.Xml;

namespace Robot_with_AOI
{
    public partial class Main_UI : Form
    {
        System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();//引用stopwatch物件
        UltraHighAccurateTimer Numerical_Timer = new UltraHighAccurateTimer();
        public Main_UI()
        {
            InitializeComponent();
        }
        private static System.Timers.Timer CTimer;
        private void Form1_Load(object sender, EventArgs e)
        {
            Numerical_Timer.Interval = 4;
            Numerical_Timer.Tick += new UltraHighAccurateTimer.ManualTimerEventHandler(Numerical_Timer_Tick);
            CTimer = new System.Timers.Timer(3000);
            CTimer.Elapsed += timeout_Tick;
            CTimer.AutoReset = true;
        }

        #region RZA2M
        byte[] rxBytes = new byte[200]; //uart rx buffer
        byte[] rxBuf = new byte[200]; //uart rx buffer
        byte tx = new byte();
        bool Position_Flag = false;
        bool Timeout_Flag = false;
        int P1_X = 0, P1_Y = 0;
        double P1_angle = 0.0;
        double J6_rotate_angle = 0;
        bool Rotate_Flag = false;
        #endregion

        #region  Status Flag
        byte TwinCAT_Flag = 0;
        byte CCD_Flag = 0;
        byte CCDTri_flag = 0;
        byte Grip_Flag = 0;
        byte UART_Flag = 0;
        bool grip1_flag = false;
        bool grip_ok_flag = false;
        byte MySQL_connect_flag = 0;
        #endregion

        #region CCD Parameter
        int CCD_errcount = 0;
        Image<Gray, byte> CCD_Cap_Img;
        PYLON_DEVICE_HANDLE hDev = new PYLON_DEVICE_HANDLE(); /* Handle for the pylon device. */
        PylonGrabResult_t grabResult;
        PylonBuffer<Byte> imgBuf = null;  /* Buffer used for grabbing. */
        uint numDevices;    /* Number of available devices. */
        bool isAvail;
        #endregion

        #region AOI Parameter
        int count = 0, count_fit = 0;
        PointF[] fit_circle_center = new PointF[100], inner_center = new PointF[100], smallest_center = new PointF[100], fit_ellipse_center = new PointF[100];
        PointF First_Hole_center = new PointF();
        float[] fit_circle_diameter = new float[100], inner_diameter = new float[100];
        float[] smallest_diameter = new float[100], fit_ellipse_diameter1 = new float[100], fit_ellipse_diameter2 = new float[100], fit_ellipse_angle = new float[100];
        float[] contour_perimeter = new float[100];
        float[] Area_diameter = new float[100];
        float contour_ratio = 0;
        #endregion

        #region RobotArm Para
        static public TcAdsClient tcClient;
        private int port = 11;
        int SlaveCount = 0;
        static long Group = 0x3040030;     //TwinCAT ADS Info
        /*
        0: StatusWord = 各軸馬達狀態暫存器的位址
        1: StatusWord1 = 各軸運動狀態暫存器的位址 
        2: ControlWord = 3A為軸1,5C為軸2, 7E為軸3, A0為軸4, C2為軸5, E4為軸6
        3: ModeOperation = 各軸運動模式的暫存器位址
        4: PositionActualValue = 實際位置的暫存器位址
        5: TargetPosition = 目標位置的暫存器位址
        6: TargetVelocity = 目標速度的暫存器位址
        7: Profile Acceleration = 目標加速度的暫存器位址
        8: Profile Deceleration = 目標減速度的暫存器位址
        9: Profile velocity = 位置控制的速度暫存器位址
        10:Interpolation = 內插模式目標位置位置暫存器位置,格式DINTL,Length = 4
        11:BufferSize = 設定內插Buffer大小暫存器位置,格式UDINT,Length = 4
        12:BufferFormat = 內插Buffer格式暫存器位置,格式USINT,Length = 1
        13:BufferClear = 內插Buffer清除暫存器位置,格式USINT,Length = 1
        14:BufferPosition = 內插Buffer旗標暫存器位置,格式USINT,Length = 2
        15:Velocity actual value ,格式DINT,Length = 4
        */
        //static public long[] SlaveRegister = { 0x8000003A, 0x8000003C, 0x8100003A, 0x81000058, 0x8000003E, 0x8100003C, 0x8100004C, 0x81000044, 0x81000048, 0x81000040, 0x81000059, 0x8100005D, 0x81000061, 0x81000062, 0x81000063, 0x80000042 };
        static public long devicelength = 0x0000002B;
        static public long[] SlaveRegister = new long[16];
        //static public double[] per001angle = { 29.12650909, 36.37260952, -29.12486667, -29.49105263, -28.99111304, -18.21669306 };

        //static public Int32[] HomePulse = { -80087, 241948857, -57195, -426525, 2492463, -3791189 };
        //static public Int32[] ZeroPulse = { -80087, 241948857, -57195, -426525, -116737, -3791189 };

        //static public Int32[] Poslimit = { 4739390, 245059973, 1548110, 5183928, 3189405, 2770667 };
        //static public Int32[] Neglimit = { -4872358, 237421725, -5441858, -6022672, -3478551, -10345352 };

        static public double[] per001angle = { 52.4288, 65.536, -43.6906648, -29.4912, -29.12712, -18.2044444 };

        //static public Int32[] HomePulse = { -44617, -611622, -436213, -76928, -223581, -16416 };
        //static public Int32[] ZeroPulse = { -44617, -611622, -436213, -76928, -2845021, -16416 };

        //static public Int32[] Poslimit = { 8606135, 4958938, 1966774, 5526400, 504597, 6537184 };
        //static public Int32[] Neglimit = { -8695369, -8803622, -8518986, -5680256, -6194639, -6570016 };

        static public Int32[] HomePulse = { -49941, -450094, 19719, -43234, 4021516, 6546616 };
        static public Int32[] ZeroPulse = { -49941, -450094, 19719, -43234, 1400076, 6513848 };

        static public Int32[] Poslimit = {  8600811 , 5120466,  2422705,  5560094, 4749695, 13067448 };
        static public Int32[] Neglimit = { -8700693, -8642094, -8063054, -5646562, -1949543, 39752 };

        static public long[] EDM_Node = { 0x81000176, 0x80000172, 0x81000174, 0x80000170 };//EDM node Addres


        public bool[] T_flag = { false, false, false, false, false, false };
        int JOG_flag = 0;

        static Int32 Position;	                                                                   //讀取各軸實際位置
        static Int32[] CurrentPosition = { 0, 0, 0, 0, 0, 0 };                                     //初始各軸現在位置
        static SByte ModeCSP = 0x8, ModePP = 0x1, ModePV = 0x3, ModeCSV = 0x9, ModeIP = 0x7;        //各種運動模式   //一般、位置、速度、同步速度
        static Int16 ReadyToSwitchOn = 0x6, SwitchOn = 0x7, OperationEnabled = 0xf, Fault = 0x80;  //改變狀態的值
        static UInt32 buffersize = 255;
        static UInt16 OPEN = 0;

        
        internal struct Numerical_Move
        {
            public int[] Finial_Pulse;
            public int[] Drift_Pulse;
            public int[] Drift_Step;
            public int[] Drift_Unit_Pulse;
            public int Moving_Count;
            public int[] MaxDrift_Step;
            public int[] Current_Pulse;
            public int MaxDrift_Total;
            public int[,] Target_Pulse;
        }
        Numerical_Move NM_Param = new Numerical_Move();

        internal struct EDM_Move
        {
            public int EDM_Move_Count;//reserved
            public bool EDM_Run;//reserved
            public bool EDM_Open;
            public bool EDM_Check;
            public bool EDM_Standby_Check;
        }
        EDM_Move EDM_Param = new EDM_Move();

        internal struct NC_Obj
        {
            public double[] X;
            public double[] Y;
            public double[] Z;
            public double[] A;
            public double[] C;
            public int counter;
            public double[,] Trans_Point;
        }

        internal struct AxisFeedBack
        {
            public UInt16 StatusWord;
            public int Position;
            public int Velocity;

        }

        internal struct AxisWrite
        {
            public ushort ControlWord;
            public int TargetPosition;
            public uint ProfileVelocity;
            public uint ProfileAcceleration;
            public uint ProfileDeceleration;
            public int TargetVelocity;
            public short TargetTorque;
            public ushort TouchProbeFunction;
            public uint DigitalOutputs;
            public sbyte ModesOfOperation;
            public int InterpolationPositionTarget;
            public uint ActualBufferSize;
            public byte BufferOrganisation;
            public byte BufferClear;
            public ushort BufferPosition;

        }
        AxisWrite[] axiswrite = new AxisWrite[6];
        AxisFeedBack[] axisfeedback = new AxisFeedBack[6];
        int rdLength = 0x2B * 6;
        int wdLength = 0x2B * 6;
        #endregion

        #region Connect Setting
        private void BT_ConTW_Click(object sender, EventArgs e)
        {
            try
            {
                XmlDocument x = new XmlDocument();
                string Object_Name = "";
                x.PreserveWhitespace = true;
                x.Load("1.xml");
                //Console.WriteLine(x.InnerXml);
                //RTB_Xml.Text += "Device : " + (k + 1).ToString() + "\n";
                XmlNodeList nodes = x.DocumentElement.SelectNodes("/TreeItem/EtherCAT/Slave/ProcessData/TxPdo");

                foreach (XmlNode OneNode in nodes)
                {
                    String StrNodeName = OneNode.Name.ToString();
                    foreach (XmlAttribute Attr in OneNode.Attributes)
                    {
                        String StrValue = OneNode.Attributes[Attr.Name.ToString()].Value;
                        if (StrValue == "3")
                        {
                            XmlNodeList TxPdo_object = OneNode.SelectNodes("Entry");
                            Group = Convert.ToInt32(TxPdo_object[0].SelectSingleNode("AdsInfo/IndexGroup").InnerText);
                            LB_GrpIdx.Text = "GrpIdx : 0x" + Group.ToString("X8");
                            for (int j = 0; j < TxPdo_object.Count; j++)
                            {
                                Object_Name = TxPdo_object[j].SelectSingleNode("Name").InnerText;
                                switch(Object_Name)
                                {
                                    case "Status word":
                                        SlaveRegister[0] = Convert.ToInt64(TxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Status word 1":
                                        SlaveRegister[1] = Convert.ToInt64(TxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    
                                    case "Position actual value":
                                        SlaveRegister[4] = Convert.ToInt64(TxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    
                                    case "Velocity actual value":
                                        SlaveRegister[15] = Convert.ToInt64(TxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                }
                            }
                        }
                    }
                }
                nodes = x.DocumentElement.SelectNodes("/TreeItem/EtherCAT/Slave/ProcessData/RxPdo");
                foreach (XmlNode OneNode in nodes)
                {
                    String StrNodeName = OneNode.Name.ToString();
                    foreach (XmlAttribute Attr in OneNode.Attributes)
                    {
                        String StrValue = OneNode.Attributes[Attr.Name.ToString()].Value;
                        if (StrValue == "2")
                        {
                            XmlNodeList RxPdo_object = OneNode.SelectNodes("Entry");
                            for (int j = 0; j < RxPdo_object.Count; j++)
                            {
                                Object_Name = RxPdo_object[j].SelectSingleNode("Name").InnerText;
                                switch (Object_Name)
                                {
                                    case "Control word":
                                        SlaveRegister[2] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Modes of operation":
                                        SlaveRegister[3] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Target position":
                                        SlaveRegister[5] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Target velocity":
                                        SlaveRegister[6] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Profile acceleration":
                                        SlaveRegister[7] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Profile deceleration":
                                        SlaveRegister[8] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Profile velocity":
                                        SlaveRegister[9] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Interpolation Position Target":
                                        SlaveRegister[10] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Actual buffer size":
                                        SlaveRegister[11] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Buffer organisation":
                                        SlaveRegister[12] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Buffer clear":
                                        SlaveRegister[13] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                    case "Buffer position":
                                        SlaveRegister[14] = Convert.ToInt64(RxPdo_object[j].SelectSingleNode("AdsInfo/IndexOffset").InnerText) & 0x00000000ffffffff;
                                        break;
                                }
                            }
                        }
                    }
                }

                tcClient = new TcAdsClient();
                port = 11;
                tcClient.Connect(port);           ///開始連線
                long index = 0x800005FA;    //Group為TwinCAT ADS Info、0x800005FA為slave count的位址
                Boolean Flag = true;
                SlaveCount = 0;
                while (Flag)
                {
                    try  ////讀取SLAVE數量
                    {

                        ushort read_data = (ushort)tcClient.ReadAny(Group, index, typeof(ushort));       //將資料從TwinCAT讀取
                        Console.Write(read_data);
                        SlaveCount = read_data;
                        Flag = false;

                    }
                    catch (Exception)
                    {
                        //MessageBox.Show(ex.Message, "Connect Erro" + Convert.ToString(num));
                        Flag = false;
                    }
                }
                LB_DeviceC.Text = "Device Count : " + Convert.ToString(SlaveCount);

                #region Enable Jog Mode
                
                for (int axis = 0; axis < 6; axis++)
                {
                    JOG_flag = 1;
                    tcClient.WriteAny(Group, SlaveRegister[3] + axis * devicelength, ModeCSV);
                    bool StartFlag = true;
                    while (StartFlag)
                    {
                        int Status = (Int16)tcClient.ReadAny(Group, SlaveRegister[0] + axis * devicelength, typeof(Int16));
                        switch (Status & 0x4F)
                        {
                            case 64:
                                //Switch on Disabled -> Ready to Switch on
                                tcClient.WriteAny(Group, SlaveRegister[2] + axis * devicelength, ReadyToSwitchOn);
                                //狀態顯示
                                Status = (Int16)tcClient.ReadAny(Group, SlaveRegister[0] + axis * devicelength, typeof(Int16));
                                break;
                            case 1:
                                //Ready to Switch on -> SwitchOn
                                tcClient.WriteAny(Group, SlaveRegister[2] + axis * devicelength, SwitchOn);
                                //狀態顯示
                                Status = (Int16)tcClient.ReadAny(Group, SlaveRegister[0] + axis * devicelength, typeof(Int16));
                                break;
                            case 3:
                                //讀取實際位置至目標位置
                                Position = (Int32)tcClient.ReadAny(Group, SlaveRegister[4] + axis * devicelength, typeof(Int32));
                                tcClient.WriteAny(Group, SlaveRegister[5] + axis * devicelength, Position);
                                //SwitchOn -> OperationEnabled
                                tcClient.WriteAny(Group, SlaveRegister[2] + axis * devicelength, OperationEnabled);
                                //狀態顯示
                                Status = (Int16)tcClient.ReadAny(Group, SlaveRegister[0] + axis * devicelength, typeof(Int16));
                                break;
                            case 8:
                                //清除錯誤
                                tcClient.WriteAny(Group, SlaveRegister[2] + axis * devicelength, Fault);
                                //狀態顯示
                                Status = (Int16)tcClient.ReadAny(Group, SlaveRegister[0] + axis * devicelength, typeof(Int16));
                                break;
                            case 7:
                                StartFlag = false;
                                //狀態顯示
                                Status = (Int16)tcClient.ReadAny(Group, SlaveRegister[0] + axis * devicelength, typeof(Int16));
                                CurrentPosition[axis] = Position;      //紀錄現在位置
                                break;
                            default:
                                StartFlag = false;
                                //狀態顯示
                                //"異常";
                                break;
                        }
                    }
                    
                    //設定加速度
                    tcClient.WriteAny(Group, SlaveRegister[7] + devicelength * axis, 2000000);
                    //設定減速度
                    tcClient.WriteAny(Group, SlaveRegister[8] + devicelength * axis, 2000000);
                    //設定速度
                    tcClient.WriteAny(Group, SlaveRegister[9] + devicelength * axis, 200000);
                    
                    LB_Mode.Text = "Mode : ModeCSV";
                }
                #endregion

                MessageBox.Show("連線成功!");
                BT_ConTW.Enabled = false;
                BT_DisTW.Enabled = true;
                GB_NM_Move.Enabled = false;
                GB_EDM.Enabled = false;
                GB_JOG.Enabled = true;
                BT_Mode.Enabled = true;
                Position_Timer.Start();
                TwinCAT_Flag = 1;
            }
            catch (Exception err)
            {
                MessageBox.Show(err.Message, "Connect Error");
            }
        }

        private void BT_DisTW_Click(object sender, EventArgs e)
        {
            try
            {
                for (int Axis = 0; Axis < 6; Axis++)
                {
                    axiswrite[Axis].ControlWord = 0x01;
                }
                AdsWrite();
                Position_Timer.Stop();
                MessageBox.Show("連線中斷!");
                BT_ConTW.Enabled = true;
                BT_DisTW.Enabled = false;
                GB_NM_Move.Enabled = false;
                GB_EDM.Enabled = false;
                GB_JOG.Enabled = false;
                TwinCAT_Flag = 0;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        private void BT_CCD_Connect_Click(object sender, EventArgs e)
        {
            try
            {
                CCD_Setting();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        private void BT_CCD_Disconnect_Click(object sender, EventArgs e)
        {
            try
            {
                Pylon.DeviceClose(hDev);
                MessageBox.Show("CCD Disconnect Successful!");
                BT_CCD_Connect.Enabled = true;
                BT_CCD_Disconnect.Enabled = false;
                BT_CCD_Start.Enabled = false;
                BT_CCD_Stop.Enabled = false;
                CCD_Flag = 0;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        private void BT_Connect_Grip_Click(object sender, EventArgs e)
        {
            try
            {
                EG_Control_API.DetectConnect();
                Grip_Form f = new Grip_Form(); //產生Form的物件，才可以使用它所提供的Method
                f.Owner = this; //重要的一步，主要是使Form的Owner指針指向Form1  
                f.ShowDialog(this);
                //EG_Control_API.ResetMotion();
                if (EG_Control_API.DetectConnect() == 0)
                {
                    EG_Control_API.ResetMotion();
                    MessageBox.Show("Grip Connect Successful!");
                    BT_Connect_Grip.Enabled = false;
                    BT_Disconnect_Grip.Enabled = true;
                    BT_Grip_Open.Enabled = true;
                    BT_Grip_Close.Enabled = true;
                    Grip_Flag = 1;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        private void BT_Disconnect_Grip_Click(object sender, EventArgs e)
        {
            try
            {
                EG_Control_API.StopMotion();
                EG_Control_API.CloseConnect();
                MessageBox.Show("Grip Disconnect Successful!");
                BT_Connect_Grip.Enabled = true;
                BT_Disconnect_Grip.Enabled = false;
                BT_Grip_Open.Enabled = false;
                BT_Grip_Close.Enabled = false;
                Grip_Flag = 0;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        private void BT_Connect_UART_Click(object sender, EventArgs e)
        {
            UART f = new UART(); //產生Form的物件，才可以使用它所提供的Method
            f.Owner = this; //重要的一步，主要是使Form的Owner指針指向Form1  
            f.ShowDialog(this);
            if (this.serialPort1.IsOpen)
            {
                MessageBox.Show("UART Connect Successful!");
                BT_Connect_UART.Enabled = false;
                BT_Disconnect_UART.Enabled = true;
                UART_Flag = 1;
            }
        }

        private void BT_Disconnect_UART_Click(object sender, EventArgs e)
        {
            this.serialPort1.Close();
            if (!this.serialPort1.IsOpen)
            {
                MessageBox.Show("UART Connect Successful!");
                BT_Connect_UART.Enabled = true;
                BT_Disconnect_UART.Enabled = false;
                UART_Flag = 0;
            }
        }
        #endregion

        #region Timer
        private void CCD_timer_Tick(object sender, EventArgs e)
        {
            /*  Grab one single frame from stream channel 0. The
                camera is set to "single frame" acquisition mode.
                Wait up to 500 ms for the image to be grabbed.
                If imgBuf is null a buffer is automatically created with the right size.*/
            if (!Pylon.DeviceGrabSingleFrame(hDev, 0, ref imgBuf, out grabResult, 500))
            {
                /* Timeout occurred. */
                Console.WriteLine("Frame {0}: timeout.", CCD_errcount++);
            }
            CCD_Cap_Img = new Image<Gray, byte>(grabResult.SizeX, grabResult.SizeY);
            CCD_Cap_Img.Bytes = imgBuf.Array;
            IB_1.Image = CCD_Cap_Img;
            imgBuf = null;
            GC.Collect();
        }

        private void Position_Timer_Tick(object sender, EventArgs e)
        {
            double[] CurrentAngle = { 0, 0, 0, 0, 0, 0 };
            for (int axis = 0; axis < 6; axis++)
            {
                CurrentPosition[axis] = (Int32)tcClient.ReadAny(Group, SlaveRegister[4] + axis * devicelength, typeof(Int32));
                CurrentAngle[axis] = PulseToAngle(CurrentPosition[axis], axis);
            }
            TB_J1Angle.Text = CurrentAngle[0].ToString("#0.0000");
            TB_J2Angle.Text = CurrentAngle[1].ToString("#0.0000");
            TB_J3Angle.Text = CurrentAngle[2].ToString("#0.0000");
            TB_J4Angle.Text = CurrentAngle[3].ToString("#0.0000");
            TB_J5Angle.Text = CurrentAngle[4].ToString("#0.0000");
            TB_J6Angle.Text = CurrentAngle[5].ToString("#0.0000");
            ForwardKinematics(CurrentAngle);

            TB_J1Encoder.Text = CurrentPosition[0].ToString();
            TB_J2Encoder.Text = CurrentPosition[1].ToString();
            TB_J3Encoder.Text = CurrentPosition[2].ToString();
            TB_J4Encoder.Text = CurrentPosition[3].ToString();
            TB_J5Encoder.Text = CurrentPosition[4].ToString();
            TB_J6Encoder.Text = CurrentPosition[5].ToString();
            
            #region 放電加工Input
            LB_Input_E.Text = "●";
            LB_Input_E2.Text = "✔";
            UInt16 ii = 0;
            UInt16 jj = 0;
            ii = (UInt16)tcClient.ReadAny(Group, EDM_Node[1], typeof(UInt16)); // Node
            jj = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16)); // Robot
            if (ii == 0)//判斷物件
            {
                LB_Input_E.ForeColor = Color.Red;
            }
            else
            {
                LB_Input_E.ForeColor = Color.Green;
            }
            if (ii > 1)
            {
                ii = 0;
            }

            if (jj == 0)//判斷物件
            {
                LB_Input_E2.ForeColor = Color.Red;
            }
            else
            {
                LB_Input_E2.ForeColor = Color.Green;
            }
            if (jj > 1)
            {
                jj = 0;
            }
            Thread.Sleep(50);
            #endregion
        }

        private void Numerical_Timer_Tick(object sender)
        {
            if (NM_Param.Moving_Count < NM_Param.MaxDrift_Total)
            {
                for (int Axis = 0; Axis < 6; Axis++)
                {
                    axiswrite[Axis].TargetPosition = Convert.ToInt32(NM_Param.Target_Pulse[Axis, NM_Param.Moving_Count]);
                }
                AdsWrite();
            }
            else
            {
                for (int Axis = 0; Axis < 6; Axis++)
                {
                    axiswrite[Axis].TargetPosition = Convert.ToInt32(NM_Param.Target_Pulse[Axis, NM_Param.MaxDrift_Total - 1]);
                }
                AdsWrite();
                NM_Param.Moving_Count = 0;
                this.Invoke((MethodInvoker)delegate ()
                {           
                        GB_NM_Move.Enabled = true;
                 });
                if (EDM_Param.EDM_Open)
                {
                    EDM_Param.EDM_Check = false; //false 確定OK關掉迴圈
                }
                Numerical_Timer.Stop();
            }
            NM_Param.Moving_Count++;
        }

        private void timeout_Tick(object source, ElapsedEventArgs e)
        {
            Timeout_Flag = true;
            this.Invoke((MethodInvoker)delegate ()
            {
                TB_Consoletext.Text = "Timeout...Repositioning.";
            });
            //Console.WriteLine("Timeout...Repositioning.");
            //MessageBox.Show("Timeout !");
        }
        #endregion

        #region Kinematics
        private void ForwardKinematics(object q)
        {
            Double[] Q = (double[])q;       //各軸角度
            double[] temp_Q = new double[6];
            for (int Axis = 0; Axis < 6; Axis++)
            {
                Q[Axis] = Q[Axis] * Math.PI / 180;
                temp_Q[Axis] = Q[Axis];
            }
            Q[1] = Q[1] + Math.PI / 2;
            double R11, R21, R31, R12, R22, R32, R13, R23, R33;
            double show_phi = 0, show_theta = 0, show_psi = 0;
            double show_x, show_y, show_z;

            for (int Axis = 0; Axis < 6; Axis++)
            {
                Q[Axis] = temp_Q[Axis];
            }
            Q[0] = Q[0] + Math.PI / 2;
            Q[1] = Q[1] + Math.PI / 2;
            show_x = 0.03 * Math.Cos(Q[0]) + 0.44 * Math.Cos(Q[0]) * Math.Cos(Q[1]) + 0.2665 * Math.Cos(Q[4]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[0]) * Math.Cos(Q[2]) * Math.Sin(Q[1])) + 0.2665 * Math.Sin(Q[4]) * (Math.Sin(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]))) + 0.04 * Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) + 0.438 * Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Sin(Q[2]) + 0.438 * Math.Cos(Q[0]) * Math.Cos(Q[2]) * Math.Sin(Q[1]) - 0.04 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]);
            show_y = 0.03 * Math.Sin(Q[0]) + 0.44 * Math.Cos(Q[1]) * Math.Sin(Q[0]) - 0.2665 * Math.Sin(Q[4]) * (Math.Cos(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) - 1.0 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0]))) + 0.2665 * Math.Cos(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[0]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[0]) * Math.Sin(Q[1])) - 0.04 * Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) + 0.04 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0]) + 0.438 * Math.Cos(Q[1]) * Math.Sin(Q[0]) * Math.Sin(Q[2]) + 0.438 * Math.Cos(Q[2]) * Math.Sin(Q[0]) * Math.Sin(Q[1]);
            show_z = 0.44 * Math.Sin(Q[1]) - 0.2665 * Math.Cos(Q[4]) * (Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Sin(Q[1]) * Math.Sin(Q[2])) - 0.438 * Math.Cos(Q[1]) * Math.Cos(Q[2]) + 0.04 * Math.Cos(Q[1]) * Math.Sin(Q[2]) + 0.04 * Math.Cos(Q[2]) * Math.Sin(Q[1]) + 0.438 * Math.Sin(Q[1]) * Math.Sin(Q[2]) + 0.2665 * Math.Cos(Q[3]) * Math.Sin(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[1]));
            //710-GB
            //show_x = 0.03 * Math.Cos(Q[0]) + 0.34 * Math.Cos(Q[0]) * Math.Cos(Q[1]) + 0.0865 * Math.Cos(Q[4]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[0]) * Math.Cos(Q[2]) * Math.Sin(Q[1])) + 0.0865 * Math.Sin(Q[4]) * (Math.Sin(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]))) + 0.04 * Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) + 0.338 * Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Sin(Q[2]) + 0.338 * Math.Cos(Q[0]) * Math.Cos(Q[2]) * Math.Sin(Q[1]) - 0.04 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]);
            //show_y = 0.03 * Math.Sin(Q[0]) + 0.34 * Math.Cos(Q[1]) * Math.Sin(Q[0]) - 0.0865 * Math.Sin(Q[4]) * (Math.Cos(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) - 1.0 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0]))) + 0.0865 * Math.Cos(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[0]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[0]) * Math.Sin(Q[1])) - 0.04 * Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) + 0.04 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0]) + 0.338 * Math.Cos(Q[1]) * Math.Sin(Q[0]) * Math.Sin(Q[2]) + 0.338 * Math.Cos(Q[2]) * Math.Sin(Q[0]) * Math.Sin(Q[1]);
            //show_z = 0.34 * Math.Sin(Q[1]) - 0.0865 * Math.Cos(Q[4]) * (Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Sin(Q[1]) * Math.Sin(Q[2])) - 0.338 * Math.Cos(Q[1]) * Math.Cos(Q[2]) + 0.04 * Math.Cos(Q[1]) * Math.Sin(Q[2]) + 0.04 * Math.Cos(Q[2]) * Math.Sin(Q[1]) + 0.338 * Math.Sin(Q[1]) * Math.Sin(Q[2]) + 0.0865 * Math.Cos(Q[3]) * Math.Sin(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[1]));

            TB_NM_feedbackX.Text = (show_x * 1000).ToString("#0.0000");
            TB_NM_feedbackY.Text = (show_y * 1000).ToString("#0.0000");
            TB_NM_feedbackZ.Text = (show_z * 1000).ToString("#0.0000");

            R11 = Math.Sin(Q[5]) * (Math.Cos(Q[3]) * Math.Sin(Q[0]) - 1.0 * Math.Sin(Q[3]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]))) + Math.Cos(Q[5]) * (Math.Cos(Q[4]) * (Math.Sin(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]))) - 1.0 * Math.Sin(Q[4]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[0]) * Math.Cos(Q[2]) * Math.Sin(Q[1])));
            R21 = -1.0 * Math.Cos(Q[5]) * (Math.Cos(Q[4]) * (Math.Cos(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) - 1.0 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0]))) + Math.Sin(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[0]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[0]) * Math.Sin(Q[1]))) - 1.0 * Math.Sin(Q[5]) * (Math.Cos(Q[0]) * Math.Cos(Q[3]) - 1.0 * Math.Sin(Q[3]) * (Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) - 1.0 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0])));
            R31 = Math.Cos(Q[5]) * (Math.Sin(Q[4]) * (Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Sin(Q[1]) * Math.Sin(Q[2])) + Math.Cos(Q[3]) * Math.Cos(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[1]))) - 1.0 * Math.Sin(Q[3]) * Math.Sin(Q[5]) * (Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[1]));

            R12 = Math.Cos(Q[5]) * (Math.Cos(Q[3]) * Math.Sin(Q[0]) - 1.0 * Math.Sin(Q[3]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]))) - 1.0 * Math.Sin(Q[5]) * (Math.Cos(Q[4]) * (Math.Sin(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]))) - 1.0 * Math.Sin(Q[4]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[0]) * Math.Cos(Q[2]) * Math.Sin(Q[1])));
            R22 = Math.Sin(Q[5]) * (Math.Cos(Q[4]) * (Math.Cos(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) - 1.0 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0]))) + Math.Sin(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[0]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[0]) * Math.Sin(Q[1]))) - 1.0 * Math.Cos(Q[5]) * (Math.Cos(Q[0]) * Math.Cos(Q[3]) - 1.0 * Math.Sin(Q[3]) * (Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) - 1.0 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0])));
            R32 = -1.0 * Math.Sin(Q[5]) * (Math.Sin(Q[4]) * (Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Sin(Q[1]) * Math.Sin(Q[2])) + Math.Cos(Q[3]) * Math.Cos(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[1]))) - 1.0 * Math.Cos(Q[5]) * Math.Sin(Q[3]) * (Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[1]));

            R13 = Math.Cos(Q[4]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[0]) * Math.Cos(Q[2]) * Math.Sin(Q[1])) + Math.Sin(Q[4]) * (Math.Sin(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Cos(Q[0]) * Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Cos(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2])));
            R23 = Math.Cos(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[0]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[0]) * Math.Sin(Q[1])) - 1.0 * Math.Sin(Q[4]) * (Math.Cos(Q[0]) * Math.Sin(Q[3]) + Math.Cos(Q[3]) * (Math.Sin(Q[0]) * Math.Sin(Q[1]) * Math.Sin(Q[2]) - 1.0 * Math.Cos(Q[1]) * Math.Cos(Q[2]) * Math.Sin(Q[0])));
            R33 = Math.Cos(Q[3]) * Math.Sin(Q[4]) * (Math.Cos(Q[1]) * Math.Sin(Q[2]) + Math.Cos(Q[2]) * Math.Sin(Q[1])) - 1.0 * Math.Cos(Q[4]) * (Math.Cos(Q[1]) * Math.Cos(Q[2]) - 1.0 * Math.Sin(Q[1]) * Math.Sin(Q[2]));

            //if (TB_Theta_Flag.Text == "0")
            //{
            //    show_theta = Math.Atan2(R13, Math.Sqrt((R11 * R11) + (R12 * R12)));
            //    TB_FOA_feedbackTheta.Text = (show_theta * 180 / Math.PI).ToString("#0.0000");
            //}
            //else
            //{
            //    show_theta = Math.Atan2(R13, -Math.Sqrt((R11 * R11) + (R12 * R12)));
            //    TB_FOA_feedbackTheta.Text = (show_theta * 180 / Math.PI).ToString("#0.0000");
            //}
            show_theta = Math.Atan2(R13, Math.Sqrt((R11 * R11) + (R12 * R12)));
            TB_NM_feedbackTheta.Text = (show_theta * 180 / Math.PI).ToString("#0.0000");

            if (Math.Abs(show_theta) >= 1.57062173 && (Math.Abs(show_theta) <= 1.57097092))
            {
                if (show_theta > 1)
                {
                    show_theta = 1.57062173;
                    show_phi = Math.Atan2(-(R23 / Math.Cos(show_theta)), (R33 / Math.Cos(show_theta)));
                    show_psi = Math.Atan2(-R12 / Math.Cos(show_theta), R11 / Math.Cos(show_theta));
                    //show_psi = 0;
                    //show_phi = Math.Atan2(R21, -R31);
                }
                else
                {
                    show_theta = -1.57062173;
                    show_phi = Math.Atan2(-(R23 / Math.Cos(show_theta)), (R33 / Math.Cos(show_theta)));
                    show_psi = Math.Atan2(-R12 / Math.Cos(show_theta), R11 / Math.Cos(show_theta));
                    //show_psi = 0;
                    //show_phi = -Math.Atan2(R21, -(R31));
                }

            }
            else
            {
                show_phi = Math.Atan2(-(R23 / Math.Cos(show_theta)), (R33 / Math.Cos(show_theta)));
                show_psi = Math.Atan2(-R12 / Math.Cos(show_theta), R11 / Math.Cos(show_theta));
            }
            if (show_phi > 1.57)
            {
                show_phi = -2 * Math.PI + show_phi;
            }
            if (show_psi > 1.57)
            {
                show_psi = -2 * Math.PI + show_psi;
            }
            TB_NM_feedbackPhi.Text = (show_phi * 180 / Math.PI).ToString("#0.0000");
            TB_NM_feedbackPsi.Text = (show_psi * 180 / Math.PI).ToString("#0.0000");
        }

        private double PulseToAngle(Int32 Pulse, int Joint)
        {
            double Angle = 0;
            Angle = ((Pulse - ZeroPulse[Joint]) / per001angle[Joint] / 1000);
            if (Joint == 4)
            {
                ZeroPulse[5] = 6513848 + Convert.ToInt32((Angle) * -364.1);
            }
            return Angle;
        }

        private Int32 AngleToPulse(double Angle, int Joint)
        {
            Int32 Pulse = 0;
            if (Joint == 4)
            {
                ZeroPulse[5] = 6513848 + Convert.ToInt32((Angle) * -364.1);
            }
            Pulse = (Int32)(ZeroPulse[Joint] + (Angle * 1000 * per001angle[Joint]));
            return Pulse;
        }

        private void NM_Feedback()
        {
            AdsReadFeedBack();
            NM_Param.Current_Pulse = new int[6];
            for (int Axis = 0; Axis < 6; Axis++)
            {
                NM_Param.Current_Pulse[Axis] = axisfeedback[Axis].Position;
            }
        }

        private void BT_Numerical_Move_Click(object sender, EventArgs e)
        {
            try
            {
                double X = 0, Y = 0, Z = 0, theta = 0, phi = 0, psi = 0;
                GB_NM_Move.Enabled = false;
                X = Convert.ToDouble(TB_NM_X.Text);
                Y = Convert.ToDouble(TB_NM_Y.Text);
                Z = Convert.ToDouble(TB_NM_Z.Text);
                phi = Convert.ToDouble(TB_NM_Phi.Text) * Math.PI / 180;
                theta = Convert.ToDouble(TB_NM_Theta.Text) * Math.PI / 180;
                psi = Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180;
                NM_Move(X, Y, Z, phi, theta, psi);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }


        private void BT_NM_Home_Click(object sender, EventArgs e)
        {
            try
            {
                GB_NM_Move.Enabled = false;
                NM_Move(0, 468, 213.5, -Math.PI, 0, -Math.PI / 2);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        private void BT_Path_Click(object sender, EventArgs e)
        {
            AOI_Path();
        }

        private void AOI_Path()
        {
            this.Invoke((MethodInvoker)delegate ()
            {
                //First hole position
                TB_NM_X.Text = "10";
                TB_NM_Y.Text = "754";
                TB_NM_Z.Text = "296";
                TB_NM_Phi.Text = "-157.0840";
                TB_NM_Theta.Text = "83.0655";
                TB_NM_Psi.Text = "-90";
            });
            while (TB_NM_Psi.Text != "-90") ;
            Thread t = new Thread(Path_Task);
            t.IsBackground = true;
            t.Start();
        }

        static string server_IP = "140.125.49.210";
        static string str = "SERVER=" + server_IP + ";DATABASE=cps_apm_aoi;PORT=3307;UID=CPS_User;";
        MySqlConnection connection = new MySqlConnection(str);
        private void BT_MySQL_Connect_Click(object sender, EventArgs e)
        {
            try
            {
                connection.Open();
                LB_MySQL_Status.Text = "Status : OK, IP : 140.125.49.210";
                MessageBox.Show("Connect Success");
                MySQL_connect_flag = 1;
                connection.Close();
            }
            catch (Exception ex)
            {
                LB_MySQL_Status.Text = "Status : Error";
                MessageBox.Show(ex.Message);
            }
        }
        NC_Obj NC_Obj_P = new NC_Obj();
        private void BT_NCRead_Click(object sender, EventArgs e)
        {
            int counter = 0;
            string line, line_check = "";
            try
            {
                using (OpenFileDialog ofd = new OpenFileDialog())
                    if (ofd.ShowDialog() == DialogResult.OK)
                    {
                        RTB_NC_Info.Text = "Origin Data : \n";
                        StreamReader file = new StreamReader(ofd.FileName);
                        while ((line = file.ReadLine()) != null)
                        {
                            if (line.Length > 3)
                            {
                                line = line.Remove(3);
                            }
                            if (line == "G00")
                            {
                                counter++;
                            }
                        }
                        NC_Obj_P.X = new double[counter];
                        NC_Obj_P.Y = new double[counter];
                        NC_Obj_P.Z = new double[counter];
                        NC_Obj_P.A = new double[counter];
                        NC_Obj_P.C = new double[counter];
                        NC_Obj_P.counter = counter;
                        LB_NC_Count.Text = "Position Num : " + counter.ToString();
                        counter = 0;
                        file = new StreamReader(ofd.FileName);
                        while ((line = file.ReadLine()) != null)
                        {
                            line_check = "";
                            if (line.Length > 3)
                            {
                                line_check = line.Remove(3);
                            }
                            if (line_check == "G00")
                            {
                                string[] array = line.Split('X', 'Y', 'Z');
                                NC_Obj_P.X[counter] = Convert.ToDouble(array[1]);
                                NC_Obj_P.Y[counter] = Convert.ToDouble(array[2]);
                                NC_Obj_P.A[counter] = 48.99;
                                NC_Obj_P.C[counter] = 229.9165;
                                NC_Obj_P.Z[counter++] = Convert.ToDouble(array[3]);
                                RTB_NC_Info.Text += "X : " + array[1] + ", Y : " + array[2] + ", Z : " + array[3] + "\n";
                            }
                        }
                    }
                BT_Path.Enabled = true;
            }
            catch (Exception ex)
            {
                LB_NC_Count.Text = "Read Error";
            }
        }

        private void Path_Task()
        {
            EDM_Param.EDM_Open = true;
            this.Invoke((MethodInvoker)delegate ()
            {
                BT_Numerical_Move.Enabled = false;
                BT_NM_Home.Enabled = false;
                GB_EDM.Enabled = false;
                BT_Path.Enabled = false;
            });
            EDM_Param.EDM_Check = true;
            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), Convert.ToDouble(TB_NM_Z.Text), Convert.ToDouble(TB_NM_Phi.Text)* Math.PI / 180, Convert.ToDouble(TB_NM_Theta.Text)* Math.PI / 180, Convert.ToDouble(TB_NM_Psi.Text)* Math.PI / 180);
            //while (EDM_Param.EDM_Check) ;
            //EDM_Param.EDM_Check = true;
            AOI_Task();

            //double dx = 20, dy = 30, dz = 20, Ra = 48.99, Rz = 229.9165;
            //for (int obj_c = 0; obj_c < NC_Obj_P.counter; obj_c++)
            //{
            //    dx = NC_Obj_P.X[obj_c];
            //    dy = NC_Obj_P.Y[obj_c];
            //    dz = NC_Obj_P.Z[obj_c];
            //    Ra = NC_Obj_P.A[obj_c];
            //    Rz = NC_Obj_P.C[obj_c];
            //    Ra = 48.99 / 180 * Math.PI;
            //    Rz = 229.9165 / 180 * Math.PI;

            //    double[,] Matrix_Rz = new double[3, 3] { { Math.Cos(Rz), -Math.Sin(Rz), 0 },
            //                                         { Math.Sin(Rz),  Math.Cos(Rz), 0 },
            //                                         {            0,             0, 1 } };

            //    double[,] Matrix_Rx = new double[3, 3]{  {            1,             0,             0 },
            //                                         {            0,  Math.Cos(Ra), -Math.Sin(Ra) },
            //                                         {            0,  Math.Sin(Ra),  Math.Cos(Ra) } };

            //    double[,] Matrix_Ry = new double[3, 3]{  { Math.Cos(Ra),             0,  Math.Sin(Ra) },
            //                                         {            0,             1,             0 },
            //                                         {-Math.Sin(Ra),             0,  Math.Cos(Ra) } };


            //    double[,] Matrix_Pos = new double[3, 1] { { dx }, { dy }, { dz } };
            //    double[,] Matrix_Result = new double[3, 3];
            //    double[,] Matrix_Result_obj = new double[3, 3];

            //    NC_Obj_P.Trans_Point = new double[NC_Obj_P.counter, 3];
            //    for (int i = 0; i < 3; i++)
            //    {
            //        for (int j = 0; j < 3; j++)
            //        {
            //            for (int k = 0; k < 3; k++)
            //            {
            //                Matrix_Result[i, j] += Matrix_Rz[i, k] * Matrix_Rx[k, j];
            //            }
            //        }
            //    }

            //    for (int i = 0; i < 3; i++)
            //    {
            //        for (int j = 0; j < 1; j++)
            //        {
            //            for (int k = 0; k < 3; k++)
            //            {
            //                Matrix_Result_obj[i, j] += Matrix_Result[i, k] * Matrix_Pos[k, j];
            //            }
            //            NC_Obj_P.Trans_Point[obj_c, i] = Math.Round(Matrix_Result_obj[i, j], 4);
            //        }
            //    }
            //    this.Invoke((MethodInvoker)delegate ()
            //    {
            //        RTB_NC_Info.Text += "X : " + NC_Obj_P.Trans_Point[obj_c, 0].ToString() + ", Y : " + NC_Obj_P.Trans_Point[obj_c, 1].ToString() + ", Z : " + NC_Obj_P.Trans_Point[obj_c, 2].ToString() + "\n";
            //    });
            //}

            //Rotate_Flag = false;
            //J6_rotate_angle = 0;
            ////1th
            //while (!Position_Flag)
            //{
            //    serialPort1.Write("1");
            //    CTimer.Enabled = true;
            //    while ((!Position_Flag) && (!Timeout_Flag)) ;
            //    CTimer.Enabled = false;
            //    Timeout_Flag = false;
            //}
            //this.Invoke((MethodInvoker)delegate ()
            //{
            //    TB_Consoletext.Text = "First Positioning Completed.";
            //});
            //Position_Flag = false;
            //Affine1();

            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), Convert.ToDouble(TB_NM_Z.Text), -Math.PI, 0, -Math.PI / 2);
            //Thread.Sleep(500);

            ////2th only J6 rotate
            //while ((!Position_Flag) || (P1_angle >= 45))
            //{
            //    serialPort1.Write("1");
            //    CTimer.Enabled = true;
            //    while ((!Position_Flag) && (!Timeout_Flag)) ;
            //    CTimer.Enabled = false;
            //    Timeout_Flag = false;
            //}
            //this.Invoke((MethodInvoker)delegate ()
            //{
            //    TB_Consoletext.Text = "Second Positioning Completed.";
            //    TB_NM_Psi.Text = TB_NM_Psi.Text + P1_angle.ToString();
            //});
            //Position_Flag = false;

            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), Convert.ToDouble(TB_NM_Z.Text), -Math.PI, 0, Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180);
            //Thread.Sleep(500);

            ////3th
            //while (!Position_Flag)
            //{
            //    serialPort1.Write("1");
            //    CTimer.Enabled = true;
            //    while ((!Position_Flag) && (!Timeout_Flag)) ;
            //    CTimer.Enabled = false;
            //    Timeout_Flag = false;
            //}
            //this.Invoke((MethodInvoker)delegate ()
            //{
            //    TB_Consoletext.Text = "Third Positioning Completed.";
            //});
            //Position_Flag = false;
            //Rotate_Flag = false;
            //Affine1();
            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), 55, -Math.PI, 0, Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180);
            //Thread.Sleep(500);

            ////4th
            //while (!Position_Flag)
            //{
            //    serialPort1.Write("2");
            //    CTimer.Enabled = true;
            //    while ((!Position_Flag) && (!Timeout_Flag)) ;
            //    CTimer.Enabled = false;
            //    Timeout_Flag = false;
            //}
            //this.Invoke((MethodInvoker)delegate ()
            //{
            //    TB_Consoletext.Text = "Fourth Positioning Completed.";
            //});
            //Position_Flag = false;
            //Affine2();
            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), 55, -Math.PI, 0, Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180);


            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), -18, -Math.PI, 0, Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180);
            //EG_Control_API.RunMove(7.5, 30);
            //while (EG_Control_API.WorkState() == true) ;
            //Thread.Sleep(500);
            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), 0, -Math.PI, 0, Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180);
            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), 30, -Math.PI, 0, Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180);
            //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), 60, -Math.PI, 0, Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180);
            //EG_Control_API.RunMove(6.3, 30);
            //Thread.Sleep(200);
            //while (EG_Control_API.WorkState() == true) ;

            //NM_Move(0, 438, 393.5, -Math.PI, 0, - Math.PI / 2);

            ////
            //Joint_num_Set = 6;
            //this.Invoke((MethodInvoker)delegate ()
            //{
            //    LB_JointCount.Text = "Joint Num : " + Convert.ToString(Joint_num_Set);
            //    TB_SetJ5.Text = "2.5";
            //    TB_SetJ6.Text = "146";
            //    TB_PX.Text = "340";
            //    TB_PY.Text = "-29";
            //    TB_PZ.Text = "335";
            //});
            //while (TB_PZ.Text != "335") ;

            //three_Axis_Move(Convert.ToDouble(TB_PX.Text), Convert.ToDouble(TB_PY.Text), Convert.ToDouble(TB_PZ.Text), 4);
            //Autofocus();
            //AOI_First_Positioning();
            //Autofocus();
            //First_Hole_Affine();
            //three_Axis_Move(Convert.ToDouble(TB_PX.Text), Convert.ToDouble(TB_PY.Text), Convert.ToDouble(TB_PZ.Text), 1);
            //Autofocus();
            //AOI_Task();

            this.Invoke((MethodInvoker)delegate ()
            {
                BT_Numerical_Move.Enabled = true;
                BT_NM_Home.Enabled = true;
                GB_EDM.Enabled = true;
                BT_Path.Enabled = true;
            });
            //EDM_Param.EDM_Open = false;
            AOI_Path_WorkFlag = false;
        }

        #region Test Run 930
        bool AOI_Path_WorkFlag = false;
        private void BT_Test_Click(object sender, EventArgs e)
        {
            try
            {
                GB_NM_Move.Enabled = false;
                EDM_Param.EDM_Check = true;
                EDM_Param.EDM_Open = true; //確定到位 判斷式開啟
                Thread t = new Thread(Test_Point);
                t.IsBackground = true;
                t.Start();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }
        private void Test_Point()
        {
            double X = 0, Y = 0, Z = 0, theta = 0, phi = 0, psi = 0;

            #region Artifact 1
            #region 進行夾料
            phi = -180 * Math.PI / 180;
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = 0; Y = 468; Z = 213.5;
            NM_Move(X, Y, Z, phi, theta, psi);
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            phi = -90 * Math.PI / 180;    // 轉姿態
            X = -200; Y = 500; Z = 300;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            phi = -90 * Math.PI / 180;
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = -332.25; Y = 500; Z = -1;//夾位起點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 730; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 760; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 780; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 805.25; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            EG_Control_API.RunMove(5, 40);
            while (EG_Control_API.WorkState() == true) ;
            Thread.Sleep(100);
            X = -332.25; Y = 805.25; Z = 19;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }


            X = -332.25; Y = 805.25; Z = 39;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 805.25; Z = 59;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 500; Z = 59;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -200; Y = 500; Z = 300;//退回轉姿態點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            phi = -180 * Math.PI / 180;//回原點
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = 0; Y = 468; Z = 213.5;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            /*-------------------工件1 夾料完成--------------------------*/
            #endregion
            #region 夾料完成 準備進機台
            phi = -180 * Math.PI / 180;//轉姿態
            theta = 0 * Math.PI / 180;
            psi = 0 * Math.PI / 180;
            X = 468; Y = 0; Z = 213.5;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            phi = -180 * Math.PI / 180;//轉姿態
            theta = 90 * Math.PI / 180;
            psi = 0 * Math.PI / 180;
            X = 648; Y = 0; Z = 393.5;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 800; Y = 0; Z = 250; //進料靠近
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 250; //X軸到點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 200; //氣閥等待點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥長開

            X = 930; Y = 0; Z = 180; //下降起點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 160; //下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 140; //下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 125; //下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 115; //下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            EG_Control_API.RunMove(32, 40); //放料
            while (EG_Control_API.WorkState() == true) ;//確認夾爪完成
            Thread.Sleep(100);

            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥長閉
            Thread.Sleep(2000);
            UInt16 i = 0;
            i = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16));
            if (i == 1)  //判斷有無工件
            {
                phi = -180 * Math.PI / 180;
                theta = 90 * Math.PI / 180;
                psi = 0 * Math.PI / 180;
                X = 930; Y = 0; Z = 140; //上升
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥長閉
                Thread.Sleep(2000);
                i = 0;
                i = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16));
                if (i == 1)
                {
                    phi = -180 * Math.PI / 180;
                    theta = 90 * Math.PI / 180;
                    psi = 0 * Math.PI / 180;
                    X = 930; Y = 0; Z = 200; //Z軸退出
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 800; Y = 0; Z = 200; //X軸退出
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 648; Y = 0; Z = 393.5; //向外移動
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 500; Y = 0; Z = 393.5; //等待點
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    tcClient.WriteAny(Group, EDM_Node[0], 100); //Start
                    Thread.Sleep(200);
                    tcClient.WriteAny(Group, EDM_Node[0], 0);
                    /*-----------工件 1 進料完成 加工開始---------------*/



                }
                else
                {
                    MessageBox.Show("無工件");
                    EDM_Param.EDM_Open = false;
                    EDM_Param.EDM_Check = false;
                    GB_NM_Move.Enabled = true;
                }

            }
            else
            {
                MessageBox.Show("無工件");
                EDM_Param.EDM_Open = false;
                EDM_Param.EDM_Check = false;
                GB_NM_Move.Enabled = true;
            }
            /*-----------工件 1 進料完成 加工開始---------------*/
            UInt16 ok = 0;
            EDM_Param.EDM_Standby_Check = true;
            while (EDM_Param.EDM_Standby_Check)
            {
                ok = (UInt16)tcClient.ReadAny(Group, EDM_Node[1], typeof(UInt16));
                if (ok == 1)
                {
                    EDM_Param.EDM_Standby_Check = false;
                }
                Thread.Sleep(50);
            }
            Thread.Sleep(6000);
            #endregion
            /*----------加工完畢準備取料----------*/
            #region 加工完成 準備取件
            phi = -180 * Math.PI / 180;
            theta = 90 * Math.PI / 180;
            psi = 0 * Math.PI / 180;
            X = 648; Y = 0; Z = 393.5; //向內移動
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 800; Y = 0; Z = 200; //向內靠近
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 800; Y = 0; Z = 115; //Z軸夾取點到位
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥長閉
            Thread.Sleep(2000);
            i = 0;
            i = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16));
            if (i == 1)  //判斷有無工件
            {
                X = 840; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 860; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 880; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 900; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 920; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 930; Y = 0; Z = 115; //X軸到點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥長開
                Thread.Sleep(2000);

                EG_Control_API.RunMove(5, 40); //夾料
                while (EG_Control_API.WorkState() == true) ;//確認夾爪完成
                Thread.Sleep(100);

                X = 930; Y = 0; Z = 125; //上升
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 930; Y = 0; Z = 140; //上升
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥長關
                i = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16));
                Thread.Sleep(2000);
                if (i == 0)
                {
                    X = 930; Y = 0; Z = 160; //上升
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 930; Y = 0; Z = 180; //上升
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 930; Y = 0; Z = 200; //上升
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 930; Y = 0; Z = 250; //上升OK
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 800; Y = 0; Z = 250; //夾料退出 X軸
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 648; Y = 0; Z = 393.5;//夾料退出
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }


                    phi = -180 * Math.PI / 180;
                    theta = 0 * Math.PI / 180;
                    psi = 0 * Math.PI / 180;
                    X = 468; Y = 0; Z = 213.5;//轉姿態
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    phi = -180 * Math.PI / 180;
                    theta = 0 * Math.PI / 180;
                    psi = -90 * Math.PI / 180;
                    X = 0; Y = 468; Z = 213.5;//轉姿態回至原點
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }
                }
                else
                {
                    MessageBox.Show("無工件");
                    EDM_Param.EDM_Open = false;
                    EDM_Param.EDM_Check = false;
                    GB_NM_Move.Enabled = true;
                }
            }
            else
            {
                MessageBox.Show("無工件");
                EDM_Param.EDM_Open = false;
                EDM_Param.EDM_Check = false;
                GB_NM_Move.Enabled = true;
            }

            /*--加工完成 取料結束--*/
            #endregion

            #region AOI 檢測
            AOI_Path_WorkFlag = true;
            AOI_Path();
            while (AOI_Path_WorkFlag) ;


            phi = -180 * Math.PI / 180;
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = 0; Y = 468; Z = 213.5;//回原點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            #endregion
            #region 辨識完成 準備放料

            phi = -90 * Math.PI / 180;
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = -200; Y = 500; Z = 300;//轉姿態
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            X = -332.25; Y = 500; Z = 59;//X軸到點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 805.25; Z = 59;//Y軸到點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 805.25; Z = 39;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 805.25; Z = 29;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 805.25; Z = 19;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 805.25; Z = 9;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 805.25; Z = -1;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            EG_Control_API.RunMove(32, 40);//放料
            while (EG_Control_API.WorkState() == true) ;
            Thread.Sleep(100);

            X = -332.25; Y = 780; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 760; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 730; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -332.25; Y = 500; Z = -1;//夾位起點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            //X = -200; Y = 500; Z = 300;
            //NM_Move(X, Y, Z, phi, theta, psi);
            //EDM_Param.EDM_Check = true;
            //while (EDM_Param.EDM_Check)
            //{
            //    Thread.Sleep(50);
            //}

            //phi = -180 * Math.PI / 180;
            //theta = 0 * Math.PI / 180;
            //psi = -90 * Math.PI / 180;
            //X = 0; Y = 468; Z = 213.5;
            //NM_Move(X, Y, Z, phi, theta, psi);
            //EDM_Param.EDM_Check = true;
            //while (EDM_Param.EDM_Check)
            //{
            //    Thread.Sleep(50);
            //}
            #endregion
            //Point End---------------
            //EDM_Param.EDM_Open = false;
            //EDM_Param.EDM_Check = false;
            //GB_NM_Move.Enabled = true;

            //MessageBox.Show("OK");

            #endregion
            #region Artifact 2
            /*-------------------夾料2夾取------------------------*/
            #region 夾料2
            //phi = -180 * Math.PI / 180;
            //theta = 0 * Math.PI / 180;
            //psi = -90 * Math.PI / 180;
            //X = 0; Y = 468; Z = 213.5;
            //NM_Move(X, Y, Z, phi, theta, psi);
            //while (EDM_Param.EDM_Check)
            //{
            //    Thread.Sleep(50);
            //}

            phi = -90 * Math.PI / 180;    // 轉姿態
            X = -200; Y = 500; Z = 300;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            phi = -90 * Math.PI / 180;
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = -190.625; Y = 550; Z = -1;//夾位起點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 730; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 760; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 780; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 806.25; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            EG_Control_API.RunMove(5, 40);
            while (EG_Control_API.WorkState() == true) ;//確認夾爪完成
            Thread.Sleep(100);

            X = -190.625; Y = 806.25; Z = 19;//向上
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }


            X = -190.625; Y = 806.25; Z = 39;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 806.25; Z = 59;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 550; Z = 59;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -200; Y = 500; Z = 300;//退回轉姿態點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            phi = -180 * Math.PI / 180;//回原點
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = 0; Y = 468; Z = 213.5;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            /*---------夾料2 完成----------*/
            #endregion
            #region 工件 放置機台
            phi = -180 * Math.PI / 180;//轉姿態
            theta = 0 * Math.PI / 180;
            psi = 0 * Math.PI / 180;
            X = 468; Y = 0; Z = 213.5;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            phi = -180 * Math.PI / 180;//轉姿態
            theta = 90 * Math.PI / 180;
            psi = 0 * Math.PI / 180;
            X = 648; Y = 0; Z = 393.5;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 800; Y = 0; Z = 250; //進料靠近
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 250; //X軸到點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 200; //氣閥等待點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥長開

            X = 930; Y = 0; Z = 180; //下降起點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 160; //下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 140; //下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 125; //下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 930; Y = 0; Z = 115; //下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            EG_Control_API.RunMove(32, 40); //放料
            while (EG_Control_API.WorkState() == true) ;//確認夾爪完成
            Thread.Sleep(100);

            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥長閉
            Thread.Sleep(2000);
            i = 0;
            i = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16));
            if (i == 1)  //判斷有無工件
            {
                phi = -180 * Math.PI / 180;
                theta = 90 * Math.PI / 180;
                psi = 0 * Math.PI / 180;
                X = 930; Y = 0; Z = 140; //上升
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥長閉
                Thread.Sleep(2000);
                i = 0;
                i = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16));
                if (i == 1)
                {
                    phi = -180 * Math.PI / 180;
                    theta = 90 * Math.PI / 180;
                    psi = 0 * Math.PI / 180;
                    X = 930; Y = 0; Z = 200; //Z軸退出
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 800; Y = 0; Z = 200; //X軸退出
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 648; Y = 0; Z = 393.5; //向外移動
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }
                    X = 500; Y = 0; Z = 393.5; //等待點
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }
                    tcClient.WriteAny(Group, EDM_Node[0], 100); //Start
                    Thread.Sleep(200);
                    tcClient.WriteAny(Group, EDM_Node[0], 0);
                    /*-----------工件 1 進料完成 加工開始---------------*/
                }
                else
                {
                    MessageBox.Show("無工件");
                    EDM_Param.EDM_Open = false;
                    EDM_Param.EDM_Check = false;
                    GB_NM_Move.Enabled = true;
                }

            }
            else
            {
                MessageBox.Show("無工件");
                EDM_Param.EDM_Open = false;
                EDM_Param.EDM_Check = false;
                GB_NM_Move.Enabled = true;
            }
            ok = 0;
            EDM_Param.EDM_Standby_Check = true;
            while (EDM_Param.EDM_Standby_Check)
            {
                ok = (UInt16)tcClient.ReadAny(Group, EDM_Node[1], typeof(UInt16));
                if (ok == 1)
                {
                    EDM_Param.EDM_Standby_Check = false;
                }
                Thread.Sleep(50);
            }
            Thread.Sleep(6000);

            #endregion
            #region 加工完成 準備取件
            phi = -180 * Math.PI / 180;
            theta = 90 * Math.PI / 180;
            psi = 0 * Math.PI / 180;
            X = 648; Y = 0; Z = 393.5; //向內移動
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 800; Y = 0; Z = 200; //向內靠近
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = 800; Y = 0; Z = 115; //Z軸夾取點到位
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
            Thread.Sleep(300);
            tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥長閉
            Thread.Sleep(2000);
            i = 0;
            i = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16));
            if (i == 1)  //判斷有無工件
            {
                X = 840; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 860; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 880; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 900; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 920; Y = 0; Z = 115; //X軸接近夾點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 930; Y = 0; Z = 115; //X軸到點
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥長開
                Thread.Sleep(2000);

                EG_Control_API.RunMove(5, 40); //夾料
                while (EG_Control_API.WorkState() == true) ;//確認夾爪完成
                Thread.Sleep(100);

                X = 930; Y = 0; Z = 125; //上升
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                X = 930; Y = 0; Z = 140; //上升
                NM_Move(X, Y, Z, phi, theta, psi);
                EDM_Param.EDM_Check = true;
                while (EDM_Param.EDM_Check)
                {
                    Thread.Sleep(50);
                }

                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 111);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥打開兩下
                Thread.Sleep(300);
                tcClient.WriteAny(Group, EDM_Node[2], 0);//氣閥長關
                i = (UInt16)tcClient.ReadAny(Group, EDM_Node[3], typeof(UInt16));
                Thread.Sleep(2000);
                if (i == 0)
                {
                    X = 930; Y = 0; Z = 160; //上升
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 930; Y = 0; Z = 180; //上升
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 930; Y = 0; Z = 200; //上升
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 930; Y = 0; Z = 250; //上升OK
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 800; Y = 0; Z = 250; //夾料退出 X軸
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    X = 648; Y = 0; Z = 393.5;//夾料退出
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }


                    phi = -180 * Math.PI / 180;
                    theta = 0 * Math.PI / 180;
                    psi = 0 * Math.PI / 180;
                    X = 468; Y = 0; Z = 213.5;//轉姿態
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }

                    phi = -180 * Math.PI / 180;
                    theta = 0 * Math.PI / 180;
                    psi = -90 * Math.PI / 180;
                    X = 0; Y = 468; Z = 213.5;//轉姿態回至原點
                    NM_Move(X, Y, Z, phi, theta, psi);
                    EDM_Param.EDM_Check = true;
                    while (EDM_Param.EDM_Check)
                    {
                        Thread.Sleep(50);
                    }
                }
                else
                {
                    MessageBox.Show("無工件");
                    EDM_Param.EDM_Open = false;
                    EDM_Param.EDM_Check = false;
                    GB_NM_Move.Enabled = true;
                }
            }
            else
            {
                MessageBox.Show("無工件");
                EDM_Param.EDM_Open = false;
                EDM_Param.EDM_Check = false;
                GB_NM_Move.Enabled = true;
            }

            /*--加工完成 取料結束--*/
            #endregion

            #region AOI 檢測
            AOI_Path_WorkFlag = true;
            AOI_Path();
            while (AOI_Path_WorkFlag) ;


            phi = -180 * Math.PI / 180;
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = 0; Y = 468; Z = 213.5;//回原點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            #endregion
            #region 檢測完畢 放回工件
            phi = -90 * Math.PI / 180;
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = -200; Y = 500; Z = 300;//轉姿態
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            X = -190.625; Y = 550; Z = 59;//X軸到點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 650; Z = 59; ;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 750; Z = 59; ;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            X = -190.625; Y = 806.25; Z = 59; ;//Y軸到點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            X = -190.625; Y = 806.25; Z = 39;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 806.25; Z = 29;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 806.25; Z = 19;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 806.25; Z = 9;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 806.25; Z = -1;//下降
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            EG_Control_API.RunMove(32, 40);//放料
            while (EG_Control_API.WorkState() == true) ;
            Thread.Sleep(100);

            X = -190.625; Y = 780; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 760; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 730; Z = -1;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -190.625; Y = 550; Z = -1;//夾位起點
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            X = -200; Y = 500; Z = 300;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }

            phi = -180 * Math.PI / 180;
            theta = 0 * Math.PI / 180;
            psi = -90 * Math.PI / 180;
            X = 0; Y = 468; Z = 213.5;
            NM_Move(X, Y, Z, phi, theta, psi);
            EDM_Param.EDM_Check = true;
            while (EDM_Param.EDM_Check)
            {
                Thread.Sleep(50);
            }
            #endregion

            /*-Point End-*/
            EDM_Param.EDM_Open = false;
            EDM_Param.EDM_Check = false;
            GB_NM_Move.Enabled = true;

            MessageBox.Show("OK");
            #endregion
        }
        #endregion


        private void BT_Start_E_Click(object sender, EventArgs e)
        {
            tcClient.WriteAny(Group, EDM_Node[0], 100); //Start
            Thread.Sleep(200);
            tcClient.WriteAny(Group, EDM_Node[0], 0);

        }

        private void BT_Stop_E_Click(object sender, EventArgs e)
        {
            tcClient.WriteAny(Group, EDM_Node[0], 200); //1 Drive
            Thread.Sleep(200);
            tcClient.WriteAny(Group, EDM_Node[0], 0);
        }

        private void BT_Grip32_Click(object sender, EventArgs e)
        {
            UInt16 GEX32;
            GEX32 = Convert.ToUInt16(TB_Grip32.Text);
            if (GEX32 <= 32)
            {
                EG_Control_API.RunMove(GEX32, 40);
            }
            else
            {

            }
        }

        private void BT_Open_E_Click(object sender, EventArgs e)
        {

            if (OPEN == 0)
            {
                tcClient.WriteAny(Group, EDM_Node[2], 111);
                OPEN = 1;
            }
            else
            {
                tcClient.WriteAny(Group, EDM_Node[2], 0);
                OPEN = 0;
            }
            /*tcClient.WriteAny(Group, EDM_Node[2], 111); //OPEN
            Thread.Sleep(100);
            tcClient.WriteAny(Group, EDM_Node[2], 0);
            Thread.Sleep(100);
            tcClient.WriteAny(Group, EDM_Node[2], 111); //OPEN
            Thread.Sleep(100);
            tcClient.WriteAny(Group, EDM_Node[2], 0);*/
        }

        private void NM_Move(double X, double Y, double Z, double phi, double theta, double psi)
        {
            bool OKcheck = false;
            Int32[] pulse = { 0, 0, 0, 0, 0, 0, 0 };
            double NM_X = 0, NM_Y = 0, NM_Z = 0;
            double[] q = new double[6];
            double[] ori_q = new double[6];
            double d_angle = new double();
            double cos_q5 = new double();
            int Q5_Rotate_Flag = -1;
            /*710-GB*/
            //double p = Math.Sqrt(Math.Pow(229840, 2) + Math.Pow(27200, 2));
            //double q3_phi = Math.Atan2(27200, 229840);
            double p = Math.Sqrt(Math.Pow(385440, 2) + Math.Pow(35200, 2));
            double q3_phi = Math.Atan2(35200, 385440);
            double k1 = new double();
            double[] poly_1 = new double[4];
            double[] poly_2 = new double[4];
            double S0 = 0, S1 = 0, Sf = 0;
            int i = 0, j = 0;

            double[,] T07 = new double[4, 4] {  {Math.Cos(theta) * Math.Cos(psi), -Math.Cos(theta) * Math.Sin(psi), Math.Sin(theta), X},
                                                { Math.Cos(phi) * Math.Sin(psi) + Math.Cos(psi) * Math.Sin(phi) * Math.Sin(theta), Math.Cos(phi) * Math.Cos(psi) - Math.Sin(phi) * Math.Sin(theta) * Math.Sin(psi), -Math.Cos(theta) * Math.Sin(phi), Y},
                                                { Math.Sin(phi) * Math.Sin(psi) - Math.Cos(phi) * Math.Cos(psi) * Math.Sin(theta), Math.Cos(psi) * Math.Sin(phi) + Math.Cos(phi) * Math.Sin(theta) * Math.Sin(psi),  Math.Cos(phi) * Math.Cos(theta), Z},
                                                { 0,                                              0,                    0, 1}};
            double[,] T76 = new double[4, 4] {  { 1, 0, 0, 0 },
                                                { 0, 1, 0, 0 },
                                                { 0, 0, 1, -266.5 }, //-86.5-180
                                                { 0, 0, 0, 1 } };
            double[,] T06 = new double[4, 4];
            for (i = 0; i < 4; i++)
            {
                for (j = 0; j < 4; j++)
                {
                    for (int k = 0; k < 4; k++)
                    {
                        T06[i, j] += T07[i, k] * T76[k, j];
                    }
                    T06[i, j] = Math.Round(T06[i, j], 8, MidpointRounding.AwayFromZero);
                }
            }
            NM_X = T06[0, 3];
            NM_Y = T06[1, 3];
            NM_Z = T06[2, 3];
            try
            {
                q[0] = Math.Atan2(NM_Y, NM_X);
                k1 = NM_X * Math.Cos(q[0]) + NM_Y * Math.Sin(q[0]);
                q[2] = Math.Atan2(Math.Pow((k1 - 30), 2) + Math.Pow(NM_Z, 2) - 387044, Math.Sqrt(Math.Pow(p, 2) - Math.Pow(Math.Pow(k1 - 30, 2) + Math.Pow(NM_Z, 2) - 387044, 2))) - q3_phi;
                q[1] = Math.Atan2((440 * Math.Cos(q[2]) + 40) * NM_Z + (440 * Math.Sin(q[2]) + 438) * (k1 - 30), -(440 * Math.Sin(q[2]) + 438) * NM_Z + (440 * Math.Cos(q[2]) + 40) * (k1 - 30)) - q[2];
                cos_q5 = T06[0, 2] * Math.Cos(q[0]) * Math.Sin(q[1] + q[2]) + T06[1, 2] * Math.Sin(q[0]) * Math.Sin(q[1] + q[2]) - T06[2, 2] * Math.Cos(q[1] + q[2]);
                q[4] = Math.Atan2(Q5_Rotate_Flag * Math.Sqrt(1 - Math.Pow(cos_q5, 2)), cos_q5);
                q[3] = Math.Atan2((T06[0, 2] * Math.Sin(q[0]) - T06[1, 2] * Math.Cos(q[0])) / Math.Sin(q[4]), (T06[0, 2] * Math.Cos(q[0]) * Math.Cos(q[1] + q[2]) + T06[1, 2] * Math.Sin(q[0]) * Math.Cos(q[1] + q[2]) + T06[2, 2] * Math.Sin(q[1] + q[2])) / Math.Sin(q[4]));
                q[5] = Math.Atan2((T06[0, 1] * Math.Cos(q[0]) * Math.Sin(q[1] + q[2]) + T06[1, 1] * Math.Sin(q[0]) * Math.Sin(q[1] + q[2]) - T06[2, 1] * Math.Cos(q[1] + q[2])) / Math.Sin(q[4]), (-T06[0, 0] * Math.Cos(q[0]) * Math.Sin(q[1] + q[2]) - T06[1, 0] * Math.Sin(q[0]) * Math.Sin(q[1] + q[2]) + T06[2, 0] * Math.Cos(q[1] + q[2])) / Math.Sin(q[4]));

                if (Math.Abs(q[3] - q[5]) >= (Math.PI * 1.95))
                {
                    Q5_Rotate_Flag = Q5_Rotate_Flag * -1;
                    q[4] = Math.Atan2(Q5_Rotate_Flag * Math.Sqrt(1 - Math.Pow(cos_q5, 2)), cos_q5);
                    q[3] = Math.Atan2((T06[0, 2] * Math.Sin(q[0]) - T06[1, 2] * Math.Cos(q[0])) / Math.Sin(q[4]), (T06[0, 2] * Math.Cos(q[0]) * Math.Cos(q[1] + q[2]) + T06[1, 2] * Math.Sin(q[0]) * Math.Cos(q[1] + q[2]) + T06[2, 2] * Math.Sin(q[1] + q[2])) / Math.Sin(q[4]));
                    q[5] = Math.Atan2((T06[0, 1] * Math.Cos(q[0]) * Math.Sin(q[1] + q[2]) + T06[1, 1] * Math.Sin(q[0]) * Math.Sin(q[1] + q[2]) - T06[2, 1] * Math.Cos(q[1] + q[2])) / Math.Sin(q[4]), (-T06[0, 0] * Math.Cos(q[0]) * Math.Sin(q[1] + q[2]) - T06[1, 0] * Math.Sin(q[0]) * Math.Sin(q[1] + q[2]) + T06[2, 0] * Math.Cos(q[1] + q[2])) / Math.Sin(q[4]));
                }

                q[0] = q[0] - Math.PI / 2;
                q[1] = q[1] - Math.PI / 2;

                for (i = 0; i < 6; i++)
                {
                    q[i] = Math.Round(q[i], 15, MidpointRounding.AwayFromZero);
                    //Console.WriteLine("q" + (i + 1).ToString() + " : " + (q[i] * 180 / Math.PI).ToString());
                }
                NM_Feedback();

                for (i = 3; i < 6; i++)
                {
                    ori_q[i] = PulseToAngle(NM_Param.Current_Pulse[i], i) / 180 * Math.PI;
                    d_angle = Math.Abs(q[i] - ori_q[i]);
                    if (d_angle > Math.PI * 4 / 3)
                    {
                        if (q[i] > ori_q[i])
                        {
                            q[i] -= 2 * Math.PI;
                        }
                        else
                        {
                            q[i] += 2 * Math.PI;
                        }
                    }
                }
                pulse[0] = AngleToPulse(q[0] * 180 / Math.PI, 0);
                pulse[1] = AngleToPulse(q[1] * 180 / Math.PI, 1);
                pulse[2] = AngleToPulse(q[2] * 180 / Math.PI, 2);
                pulse[3] = AngleToPulse(q[3] * 180 / Math.PI, 3);
                pulse[4] = AngleToPulse(q[4] * 180 / Math.PI, 4);
                pulse[5] = AngleToPulse(q[5] * 180 / Math.PI, 5);

                if (pulse[0] <= Poslimit[0] && pulse[0] >= Neglimit[0])
                {
                    if (pulse[1] <= Poslimit[1] && pulse[1] >= Neglimit[1])
                    {
                        if (pulse[2] <= Poslimit[2] && pulse[2] >= Neglimit[2])
                        {
                            if (pulse[3] <= Poslimit[3] && pulse[3] >= Neglimit[3])
                            {
                                if (pulse[4] <= Poslimit[4] && pulse[4] >= Neglimit[4])
                                {
                                    if (pulse[5] <= Poslimit[5] && pulse[5] >= Neglimit[5])
                                    {
                                        OKcheck = true;
                                    }
                                    else
                                    {
                                        MessageBox.Show("J6超過極限");
                                    }
                                }
                                else
                                {
                                    MessageBox.Show("J5超過極限");
                                }
                            }
                            else
                            {
                                MessageBox.Show("J4超過極限");
                            }
                        }
                        else
                        {
                            MessageBox.Show("J3超過極限");
                        }
                    }
                    else
                    {
                        MessageBox.Show("J2超過極限");
                    }
                }
                else
                {
                    MessageBox.Show("J1超過極限");
                }

                if (OKcheck == true)
                {
                    OKcheck = false;
                    NM_Param.Finial_Pulse = new int[6];
                    NM_Param.Current_Pulse = new int[6];
                    NM_Param.Drift_Pulse = new int[6];
                    NM_Param.Drift_Step = new int[6];
                    NM_Param.Drift_Unit_Pulse = new int[6];
                    NM_Param.MaxDrift_Step = new int[6];
                    NM_Param.MaxDrift_Total = new int();
                    NM_Param.Moving_Count = new int();
                    //Target pulse
                    for (i = 0; i < 6; i++)
                    {
                        NM_Param.Finial_Pulse[i] = pulse[i];
                    }
                    NM_Feedback();
                    //Dift
                    for (i = 0; i < 6; i++)
                    {
                        NM_Param.Drift_Step[i] = Convert.ToInt32(Math.Abs(Convert.ToDouble(NM_Param.Finial_Pulse[i] - NM_Param.Current_Pulse[i]) / (per001angle[i] * 40))); //60
                        if (NM_Param.Drift_Step[i] > NM_Param.MaxDrift_Total)
                        {
                            NM_Param.MaxDrift_Total = Convert.ToInt32(NM_Param.Drift_Step[i]);
                        }
                    }
                    if ((NM_Param.MaxDrift_Total % 2) != 0)
                        NM_Param.MaxDrift_Total += 1;
                    NM_Param.Target_Pulse = new int[6, NM_Param.MaxDrift_Total];
                    NM_Param.MaxDrift_Total /= 2;
                    //All position write
                    if (NM_Param.MaxDrift_Total != 0)
                    {
                        for (i = 0; i < 6; i++)
                        {
                            S0 = NM_Param.Current_Pulse[i];
                            Sf = NM_Param.Finial_Pulse[i];
                            S1 = (S0 + Sf) / 2;
                            poly_1[0] = S0;
                            poly_1[1] = 0;
                            poly_1[2] = (12 * S1 - 3 * Sf - 9 * S0) / (4 * Math.Pow(NM_Param.MaxDrift_Total, 2));
                            poly_1[3] = (-8 * S1 + 3 * Sf + 5 * S0) / (4 * Math.Pow(NM_Param.MaxDrift_Total, 3));
                            poly_2[0] = S1;
                            poly_2[1] = (3 * Sf - 3 * S0) / (4 * NM_Param.MaxDrift_Total);
                            poly_2[2] = (-12 * S1 + 6 * Sf + 6 * S0) / (4 * Math.Pow(NM_Param.MaxDrift_Total, 2));
                            poly_2[3] = (8 * S1 - 5 * Sf - 3 * S0) / (4 * Math.Pow(NM_Param.MaxDrift_Total, 3));

                            for (j = 0; j < NM_Param.MaxDrift_Total; j++)
                            {
                                NM_Param.Target_Pulse[i, j] = Convert.ToInt32(poly_1[0] + poly_1[1] * j + poly_1[2] * j * j + poly_1[3] * j * j * j);
                            }
                            for (j = 0; j < NM_Param.MaxDrift_Total - 1; j++)
                            {
                                NM_Param.Target_Pulse[i, j + NM_Param.MaxDrift_Total] = Convert.ToInt32(poly_2[0] + poly_2[1] * j + poly_2[2] * j * j + poly_2[3] * j * j * j);
                            }
                            NM_Param.Target_Pulse[i, j + NM_Param.MaxDrift_Total] = NM_Param.Finial_Pulse[i];
                        }
                        //setting current position and set into OP
                        AdsWriteRead();
                        for (int Axis = 0; Axis < 6; Axis++)
                        {
                            axiswrite[Axis].ModesOfOperation = ModeCSP;
                            axiswrite[Axis].ControlWord = 0x0f;
                            axiswrite[Axis].TargetPosition = Convert.ToInt32(NM_Param.Current_Pulse[Axis]);
                        }
                        AdsWrite();
                        NM_Param.MaxDrift_Total *= 2;
                        Numerical_Timer.Start();
                    }
                    else
                    {
                        AdsWriteRead();
                        for (int Axis = 0; Axis < 6; Axis++)
                        {
                            axiswrite[Axis].ModesOfOperation = ModeCSP;
                            axiswrite[Axis].ControlWord = 0x0f;
                            axiswrite[Axis].TargetPosition = Convert.ToInt32(NM_Param.Finial_Pulse[Axis]);
                        }
                        AdsWrite();
                        EDM_Param.EDM_Check = false;
                        this.Invoke((MethodInvoker)delegate ()
                        {
                            GB_NM_Move.Enabled = true;
                        });

                    }
                }
                else
                {
                    GB_NM_Move.Enabled = true;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        #endregion

        #region serialport
        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            try
            {
                byte readData;
                byte RX_Flag = 0;
                int i = 0;

                while ((readData = Convert.ToByte(serialPort1.ReadByte())) != 0x0a)
                {

                    if ((RX_Flag == 1) || (RX_Flag == 2))
                    {
                        if (readData == 0x20)
                            readData = 0x30;
                        rxBytes[i++] = readData;
                    }
                    else
                    {
                        if (readData == 'M')
                        {
                            RX_Flag = 1;
                            rxBytes[i++] = readData;
                        }
                        else if (readData == 'N')
                        {
                            RX_Flag = 2;
                            rxBytes[i++] = readData;
                        }
                    }
                }
                rxBytes[i] = 0x0a;
                //serialPort1.DiscardInBuffer();
                if (RX_Flag == 1)
                {
                    RX_Flag = 0;
                    this.Invoke(new EventHandler(rxFunction));
                    serialPort1.DiscardInBuffer();
                }
                else if (RX_Flag == 2)
                {
                    RX_Flag = 0;
                    this.Invoke(new EventHandler(rxFunction));
                    serialPort1.DiscardInBuffer();
                }
            }
            //=例外處理=
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        private void rxFunction(object obj, EventArgs e)
        {
            byte RX_Buf_Count = 0;
            int RX_X = 0, RX_Y = 0;
            double RX_degree = 0.0;
            switch (rxBytes[0])
            {
                case 0x4d:      //M
                    while (rxBytes[RX_Buf_Count] != 0x0a)
                    {
                        if (rxBytes[RX_Buf_Count] == 'x')
                        {
                            RX_X = (rxBytes[RX_Buf_Count + 2] - 0x30) * 1000 + (rxBytes[RX_Buf_Count + 3] - 0x30) * 100 + (rxBytes[RX_Buf_Count + 4] - 0x30) * 10 + (rxBytes[RX_Buf_Count + 5] - 0x30);
                            P1_X = RX_X;
                            RX_Buf_Count += 5;
                        }
                        else if (rxBytes[RX_Buf_Count] == 'y')
                        {
                            RX_Y = (rxBytes[RX_Buf_Count + 2] - 0x30) * 100 + (rxBytes[RX_Buf_Count + 3] - 0x30) * 10 + (rxBytes[RX_Buf_Count + 4] - 0x30);
                            P1_Y = RX_Y;
                            RX_Buf_Count += 4;
                        }
                        else if (rxBytes[RX_Buf_Count] == 'd')
                        {
                            if (rxBytes[RX_Buf_Count + 2] == '-')
                            {
                                RX_degree = (rxBytes[RX_Buf_Count + 3] - 0x30) * 10 + (rxBytes[RX_Buf_Count + 4] - 0x30) + (double)(rxBytes[RX_Buf_Count + 6] - 0x30) / 10 + (double)(rxBytes[RX_Buf_Count + 7] - 0x30) / 100 + (double)(rxBytes[RX_Buf_Count + 8] - 0x30) / 1000;
                                RX_degree = 0 - RX_degree;
                            }
                            else if (rxBytes[RX_Buf_Count + 3] == '-')
                            {
                                RX_degree = (rxBytes[RX_Buf_Count + 4] - 0x30) + (double)(rxBytes[RX_Buf_Count + 6] - 0x30) / 10 + (double)(rxBytes[RX_Buf_Count + 7] - 0x30) / 100 + (double)(rxBytes[RX_Buf_Count + 8] - 0x30) / 1000;
                                RX_degree = 0 - RX_degree;
                            }
                            else
                            {
                                RX_degree = (rxBytes[RX_Buf_Count + 3] - 0x30) * 10 + (rxBytes[RX_Buf_Count + 4] - 0x30) + (double)(rxBytes[RX_Buf_Count + 6] - 0x30) / 10 + (double)(rxBytes[RX_Buf_Count + 7] - 0x30) / 100 + (double)(rxBytes[RX_Buf_Count + 8] - 0x30) / 1000;
                            }
                            P1_angle = RX_degree;
                        }
                        RX_Buf_Count++;
                    }
                    Position_Flag = true;
                    break;

                case 0x4e:      //N
                    while (rxBytes[RX_Buf_Count] != 0x0a)
                    {
                        if (rxBytes[RX_Buf_Count] == 'x')
                        {
                            RX_X = (rxBytes[RX_Buf_Count + 2] - 0x30) * 1000 + (rxBytes[RX_Buf_Count + 3] - 0x30) * 100 + (rxBytes[RX_Buf_Count + 4] - 0x30) * 10 + (rxBytes[RX_Buf_Count + 5] - 0x30);
                            P1_X = RX_X;
                            RX_Buf_Count += 5;
                        }
                        else if (rxBytes[RX_Buf_Count] == 'y')
                        {
                            RX_Y = (rxBytes[RX_Buf_Count + 2] - 0x30) * 100 + (rxBytes[RX_Buf_Count + 3] - 0x30) * 10 + (rxBytes[RX_Buf_Count + 4] - 0x30);
                            P1_Y = RX_Y;
                            RX_Buf_Count += 4;
                        }
                        RX_Buf_Count++;
                    }
                    Position_Flag = true;
                    break;
            }
            Array.Clear(rxBytes, 0, 200);
        }
        #endregion


        #region AdsReadWrite
        private void AdsReadFeedBack()
        {
            try
            {
                //sw_ADS.Start();
                // Get the ADS return codes and examine for errors
                AdsStream rdStream = new AdsStream(rdLength);
                tcClient.Read(Group, 0x80000047, rdStream);
                BinaryReader reader = new BinaryReader(rdStream);

                // Read the data from the ADS stream
                for (int Axis = 0; Axis < 6; Axis++)
                {
                    axisfeedback[Axis].StatusWord = reader.ReadUInt16();
                    //offset
                    reader.ReadBytes(2);
                    axisfeedback[Axis].Position = reader.ReadInt32();
                    axisfeedback[Axis].Velocity = reader.ReadInt32();
                    //offset
                    reader.ReadBytes(0x1F);
                }
                //sw_ADS.Stop();
                //Console.Write("Ads each time needs " + sw_ADS.ElapsedMilliseconds + "ms\n");
                //sw_ADS.Reset();

            }
            catch (Exception err)
            {
                MessageBox.Show(err.Message);
            }
        }

        private void AdsWriteRead()
        {
            try
            {
                //sw_ADS.Start();
                // Get the ADS return codes and examine for errors
                AdsStream wrdStream = new AdsStream(wdLength);
                //tcClient.Read(Group, 0x8100003A, wrdStream);
                tcClient.Read(Group, 0x81000047, wrdStream);
                BinaryReader writer = new BinaryReader(wrdStream);

                // Read the data from the ADS stream
                for (int Axis = 0; Axis < 6; Axis++)
                {
                    axiswrite[Axis].ControlWord = writer.ReadUInt16();
                    axiswrite[Axis].TargetPosition = writer.ReadInt32();
                    axiswrite[Axis].ProfileVelocity = writer.ReadUInt32();
                    axiswrite[Axis].ProfileAcceleration = writer.ReadUInt32();
                    axiswrite[Axis].ProfileDeceleration = writer.ReadUInt32();
                    axiswrite[Axis].TargetVelocity = writer.ReadInt32();
                    axiswrite[Axis].TargetTorque = writer.ReadInt16();
                    axiswrite[Axis].TouchProbeFunction = writer.ReadUInt16();
                    axiswrite[Axis].DigitalOutputs = writer.ReadUInt32();
                    axiswrite[Axis].ModesOfOperation = writer.ReadSByte();
                    axiswrite[Axis].InterpolationPositionTarget = writer.ReadInt32();
                    axiswrite[Axis].ActualBufferSize = writer.ReadUInt32();
                    axiswrite[Axis].BufferOrganisation = writer.ReadByte();
                    axiswrite[Axis].BufferClear = writer.ReadByte();
                    axiswrite[Axis].BufferPosition = writer.ReadUInt16();
                }
                //sw_ADS.Stop();
                //Console.Write("Ads each time needs " + sw_ADS.ElapsedMilliseconds + "ms\n");
                //sw_ADS.Reset();

            }
            catch (Exception err)
            {
                MessageBox.Show(err.Message);
            }
        }

        private void AdsWrite()
        {
            AdsStream wdStream = new AdsStream(wdLength);
            BinaryWriter Writer = new BinaryWriter(new AdsStream(wdLength));
            for (int Axis = 0; Axis < 6; Axis++)
            {
                Writer.Write(axiswrite[Axis].ControlWord);
                Writer.Write(axiswrite[Axis].TargetPosition);
                Writer.Write(axiswrite[Axis].ProfileVelocity);
                Writer.Write(axiswrite[Axis].ProfileAcceleration);
                Writer.Write(axiswrite[Axis].ProfileDeceleration);
                Writer.Write(axiswrite[Axis].TargetVelocity);
                Writer.Write(axiswrite[Axis].TargetTorque);
                Writer.Write(axiswrite[Axis].TouchProbeFunction);
                Writer.Write(axiswrite[Axis].DigitalOutputs);
                Writer.Write(axiswrite[Axis].ModesOfOperation);
                Writer.Write(axiswrite[Axis].InterpolationPositionTarget);
                Writer.Write(axiswrite[Axis].ActualBufferSize);
                Writer.Write(axiswrite[Axis].BufferOrganisation);
                Writer.Write(axiswrite[Axis].BufferClear);
                Writer.Write(axiswrite[Axis].BufferPosition);
            }

            //tcClient.Write(Group, 0x8100003A, (AdsStream)Writer.BaseStream);
            tcClient.Write(Group, 0x81000047, (AdsStream)Writer.BaseStream);
        }

        private void LB_Input_E_Click(object sender, EventArgs e)
        {

        }


        private void groupBox5_Enter(object sender, EventArgs e)
        {

        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            Autofocus();
        }
        #endregion

        #region CCD Region
        private void BT_CCD_Start_Click(object sender, EventArgs e)
        {
            try
            {
                CCD_timer.Start();
                BT_CCD_Start.Enabled = false;
                BT_CCD_Stop.Enabled = true;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        private void BT_CCD_Stop_Click(object sender, EventArgs e)
        {
            try
            {
                CCD_timer.Stop();
                BT_CCD_Start.Enabled = true;
                BT_CCD_Stop.Enabled = false;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }

        public void CCD_Setting()
        {
            try
            {
                /* Before using any pylon methods, the pylon runtime must be initialized. */
                Pylon.Initialize();

                /* Enumerate all camera devices. You must call
                PylonEnumerateDevices() before creating a device. */
                numDevices = Pylon.EnumerateDevices();

                if (0 == numDevices)
                {
                    throw new Exception("No devices found.");
                }

                /* Get a handle for the first device found.  */
                hDev = Pylon.CreateDeviceByIndex(0);

                /* Before using the device, it must be opened. Open it for configuring
                parameters and for grabbing images. */
                Pylon.DeviceOpen(hDev, Pylon.cPylonAccessModeControl | Pylon.cPylonAccessModeStream);

                /* Set the pixel format to Mono8, where gray values will be output as 8 bit values for each pixel. */
                /* ... Check first to see if the device supports the Mono8 format. */
                isAvail = Pylon.DeviceFeatureIsAvailable(hDev, "EnumEntry_PixelFormat_Mono8");

                if (!isAvail)
                {
                    /* Feature is not available. */
                    throw new Exception("Device doesn't support the Mono8 pixel format.");
                }

                /* ... Set the pixel format to Mono8. */
                Pylon.DeviceFeatureFromString(hDev, "PixelFormat", "Mono8");

                /* Disable acquisition start trigger if available. */
                isAvail = Pylon.DeviceFeatureIsAvailable(hDev, "EnumEntry_TriggerSelector_AcquisitionStart");
                if (isAvail)
                {
                    Pylon.DeviceFeatureFromString(hDev, "TriggerSelector", "AcquisitionStart");
                    Pylon.DeviceFeatureFromString(hDev, "TriggerMode", "Off");
                }

                /* Disable frame burst start trigger if available */
                isAvail = Pylon.DeviceFeatureIsAvailable(hDev, "EnumEntry_TriggerSelector_FrameBurstStart");
                if (isAvail)
                {
                    Pylon.DeviceFeatureFromString(hDev, "TriggerSelector", "FrameBurstStart");
                    Pylon.DeviceFeatureFromString(hDev, "TriggerMode", "Off");
                }

                /* Disable frame start trigger if available */
                isAvail = Pylon.DeviceFeatureIsAvailable(hDev, "EnumEntry_TriggerSelector_FrameStart");
                if (isAvail)
                {
                    Pylon.DeviceFeatureFromString(hDev, "TriggerSelector", "FrameStart");
                    Pylon.DeviceFeatureFromString(hDev, "TriggerMode", "Off");
                }

                /* For GigE cameras, we recommend increasing the packet size for better
                   performance. If the network adapter supports jumbo frames, set the packet
                   size to a value > 1500, e.g., to 8192. In this sample, we only set the packet size
                   to 1500. */
                /* ... Check first to see if the GigE camera packet size parameter is supported
                    and if it is writable. */
                isAvail = Pylon.DeviceFeatureIsWritable(hDev, "GevSCPSPacketSize");

                if (isAvail)
                {
                    /* ... The device supports the packet size feature. Set a value. */
                    Pylon.DeviceSetIntegerFeature(hDev, "GevSCPSPacketSize", 1500);
                }
                MessageBox.Show("CCD Connect Successful!");
                BT_CCD_Connect.Enabled = false;
                BT_CCD_Disconnect.Enabled = true;
                BT_CCD_Start.Enabled = true;
                BT_CCD_Stop.Enabled = false;
                CCD_Flag = 1;
            }
            catch
            {
                try
                {
                    if (hDev.IsValid)
                    {
                        /* ... Close and release the pylon device. */
                        if (Pylon.DeviceIsOpen(hDev))
                        {
                            Pylon.DeviceClose(hDev);
                        }
                        Pylon.DestroyDevice(hDev);
                    }
                }
                catch (Exception)
                {
                    /*No further handling here.*/
                }

                Pylon.Terminate();  /* Releases all pylon resources. */

                Console.Error.WriteLine("\nPress enter to exit.");
                Console.ReadLine();
                MessageBox.Show("CCD Active Error");
                //Environment.Exit(1);
            }
        }

        private void Get_CCDImg()
        {
            /*  Grab one single frame from stream channel 0. The
            camera is set to "single frame" acquisition mode.
            Wait up to 500 ms for the image to be grabbed.
            If imgBuf is null a buffer is automatically created with the right size.*/
            if (!Pylon.DeviceGrabSingleFrame(hDev, 0, ref imgBuf, out grabResult, 500))
            {
                /* Timeout occurred. */
                Console.WriteLine("Frame {0}: timeout.", CCD_errcount++);
            }
            CCD_Cap_Img = new Image<Gray, byte>(grabResult.SizeX, grabResult.SizeY);
            CCD_Cap_Img.Bytes = imgBuf.Array;
            IB_1.Image = CCD_Cap_Img;
            imgBuf = null;
            GC.Collect();
        }

        int Item_ID0 = 1, Item_ID1 = 1;
        private void BT_Hole_Detect_Click(object sender, EventArgs e)
        {
            Image<Rgb, byte> Img_C1 = null;
            Image<Rgb, byte> Img_C2 = null;
            Image<Gray, byte> Img_G1 = null;
            Image<Gray, byte> Img_G2 = null;
            Image<Gray, byte> Img_G3 = null;
            Image<Gray, byte> Img_G4 = null;

            
            count = 0;
            if (CCDTri_flag == 0)
            {
                //Get_CCDImg();
                //using (OpenFileDialog ofd = new OpenFileDialog())
                //if (ofd.ShowDialog() == DialogResult.OK)
                //{
                //    Img_G1 = new Image<Gray, byte>(ofd.FileName);
                //}
                Img_G1 = new Image<Gray, byte>("1.bmp");
            }
            else
            {
                Img_G1 = CCD_Cap_Img;
            }
            Img_G2 = Img_G1;
            Img_G3 = Img_G1.CopyBlank();
            Img_G4 = Img_G1.CopyBlank();
            Img_C1 = Img_G1.Convert<Rgb, byte>();
            Img_C2 = Img_G1.Convert<Rgb, byte>();

            Img_G2 = Img_G2.Dilate(9);
            Img_G2 = Img_G2.Erode(9);
            Img_G2 = Img_G2.SmoothGaussian(21);
            Img_G2 = Img_G2.ThresholdBinaryInv(new Gray(17), new Gray(255)); //20
            Img_G2 = Img_G2.SmoothMedian(15);
            Contour<Point> contours = Img_G2.FindContours(
            Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_NONE,
            Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_EXTERNAL);
            MCvFont font = new MCvFont();
            CvInvoke.cvInitFont(ref font, Emgu.CV.CvEnum.FONT.CV_FONT_HERSHEY_COMPLEX, 3, 3, 0, 4, Emgu.CV.CvEnum.LINE_TYPE.FOUR_CONNECTED);
            for (; contours != null; contours = contours.HNext)
            {
                contour_ratio = contours.GetMinAreaRect().size.Height / contours.GetMinAreaRect().size.Width;

                if ((contours.Area >= 8000) && (contours.Area <= 200000) && (contour_ratio < 2) && (contour_ratio > 0.5))
                {
                    First_Hole_center = contours.GetMinAreaRect().center;
                    contour_perimeter[count] = (float)(contours.Perimeter * 3.7); //2.36174
                    contour_perimeter[count] = (float)(CvInvoke.cvArcLength(contours, MCvSlice.WholeSeq, -1) * 3.7);

                    Area_diameter[count] = (float)(Math.Sqrt(contours.Area / Math.PI) * 2 * 3.7);
                    Img_C1.Draw(contours, new Rgb(Color.White), 1);
                    Img_C1.Draw((First_Hole_center.X).ToString() + "," + (First_Hole_center.Y).ToString(), ref font, new Point((int)contours.GetMinAreaRect().center.X, (int)contours.GetMinAreaRect().center.Y), new Rgb(Color.Red));
                    Img_C2.Draw(contours, new Rgb(Color.White), 2);
                    Img_G3.Draw(contours, new Gray(255), 1);

                    #region 內切圓
                    double dist = 0;
                    double maxdist = 0;
                    PointF center = new PointF(0, 0);
                    float X_left = (float)(contours.BoundingRectangle.X + contours.BoundingRectangle.Width * 0.25);
                    float X_right = (float)(contours.BoundingRectangle.X + contours.BoundingRectangle.Width * 0.75);
                    float Y_top = (float)(contours.BoundingRectangle.Y + contours.BoundingRectangle.Height * 0.25);
                    float Y_bottom = (float)(contours.BoundingRectangle.Y + contours.BoundingRectangle.Height * 0.75);

                    for (float k = X_left; k < X_right; k = k + 5)
                    {
                        for (float j = Y_top; j < Y_bottom; j = j + 5)
                        {
                            PointF pt = new PointF(k, j);
                            //ptc[0] = Point.Round(pt);
                            //newimage1.DrawPolyline(ptc, true, new Rgb(Color.Yellow), 2);
                            dist = CvInvoke.cvPointPolygonTest(contours, pt, true); //求得pt與輪廓的距離 內部正值,外部負值,線上0
                            if (dist > maxdist)
                            {
                                maxdist = dist;
                                center = pt;
                            }
                        }
                    }
                    inner_center[count] = center;
                    inner_diameter[count] = (float)maxdist;
                    Img_C1.Draw(new CircleF(inner_center[count], (float)inner_diameter[count]), new Rgb(Color.Red), 2);
                    #endregion

                    #region 外接圓
                    CvInvoke.cvMinEnclosingCircle(contours, out smallest_center[count], out smallest_diameter[count]);
                    Img_C1.Draw(new CircleF(smallest_center[count], (float)smallest_diameter[count]), new Rgb(Color.Green), 2);
                    #endregion

                    #region 橢圓擬合
                    Ellipse fittedEllipse = new Ellipse(CvInvoke.cvFitEllipse2(contours));
                    fit_ellipse_center[count] = fittedEllipse.MCvBox2D.center;
                    fit_ellipse_diameter1[count] = fittedEllipse.MCvBox2D.size.Height;
                    fit_ellipse_diameter2[count] = fittedEllipse.MCvBox2D.size.Width;
                    fit_ellipse_angle[count] = fittedEllipse.MCvBox2D.angle;
                    Img_C1.Draw(new Ellipse(fit_ellipse_center[count], new SizeF(fit_ellipse_diameter1[count], fit_ellipse_diameter2[count]), fit_ellipse_angle[count]), new Rgb(Color.SkyBlue), 2);
                    #endregion

                    count++;
                }
            }

            #region 霍夫找圓
            CircleF[][] fit_circles = Img_G3.HoughCircles(      //霍夫找圓
            new Gray(255), //Canny algorithm high threshold
                           //(the lower one will be twice smaller)
            new Gray(100),  //accumulator threshold at the center detection stage
            4,             //accumulator resolution
            200,           //偵測到任意兩圓心最小距離
            55,           //最小半徑
            75);         //最大半徑

            foreach (CircleF circle in fit_circles[0])
            {
                fit_circle_center[count_fit] = circle.Center;
                fit_circle_diameter[count_fit] = (float)(circle.Radius * 2 * 3.7);
                Img_C1.Draw(circle, new Rgb(Color.Orange), 2);
                count_fit++;
            }
            #endregion

            for (int i = 0; i < count; i++)
            {
                inner_diameter[i] = (float)(inner_diameter[i] * 2 * 3.7);
                smallest_diameter[i] = (float)(smallest_diameter[i] * 2 * 3.7);
                fit_ellipse_diameter1[i] = (float)(fit_ellipse_diameter1[i] * 3.7);
                fit_ellipse_diameter2[i] += (float)(fit_ellipse_diameter2[i] * 3.7);
            }

            #region 標準差
            TB_Hough_SD.Text = SD_Cal(fit_circle_diameter, count_fit).ToString();
            TB_Area_SD.Text = SD_Cal(Area_diameter, count).ToString();
            TB_Enclosing_SD.Text = SD_Cal(smallest_diameter, count).ToString();
            TB_Inner_SD.Text = SD_Cal(inner_diameter, count).ToString();
            TB_EllipseB_SD.Text = SD_Cal(fit_ellipse_diameter1, count).ToString();
            TB_EllipseA_SD.Text = SD_Cal(fit_ellipse_diameter2, count).ToString();
            TB_Perimeter_SD.Text = SD_Cal(contour_perimeter, count).ToString();
            #endregion

            #region 最小值
            TB_Hough_Min.Text = fit_circle_diameter.Where(q => q != 0).Min().ToString();
            TB_Area_Min.Text = Area_diameter.Where(q => q != 0).Min().ToString();
            TB_Enclosing_Min.Text = smallest_diameter.Where(q => q != 0).Min().ToString();
            TB_Inner_Min.Text = inner_diameter.Where(q => q != 0).Min().ToString();
            TB_EllipseB_Min.Text = fit_ellipse_diameter1.Where(q => q != 0).Min().ToString();
            TB_EllipseA_Min.Text = fit_ellipse_diameter2.Where(q => q != 0).Min().ToString();
            TB_Perimeter_Min.Text = contour_perimeter.Where(q => q != 0).Min().ToString();
            #endregion

            #region 最大值
            TB_Hough_Max.Text = fit_circle_diameter.Max().ToString();
            TB_Area_Max.Text = Area_diameter.Max().ToString();
            TB_Enclosing_Max.Text = smallest_diameter.Max().ToString();
            TB_Inner_Max.Text = inner_diameter.Max().ToString();
            TB_EllipseB_Max.Text = fit_ellipse_diameter1.Max().ToString();
            TB_EllipseA_Max.Text = fit_ellipse_diameter2.Max().ToString();
            TB_Perimeter_Max.Text = contour_perimeter.Max().ToString();
            #endregion

            #region 平均值
            TB_Hough_Avg.Text = (fit_circle_diameter.Sum() / count_fit).ToString();
            TB_Area_Avg.Text = (Area_diameter.Sum() / count).ToString();
            TB_Enclosing_Avg.Text = (smallest_diameter.Sum() / count).ToString();
            TB_Inner_Avg.Text = (inner_diameter.Sum() / count).ToString();
            TB_EllipseB_Avg.Text = (fit_ellipse_diameter1.Sum() / count).ToString();
            TB_EllipseA_Avg.Text = (fit_ellipse_diameter2.Sum() / count).ToString();
            TB_Perimeter_Avg.Text = (contour_perimeter.Sum() / count).ToString();
            #endregion

            count = 0;
            IB_1.Image = Img_G1;
            IB_2.Image = Img_C1;
            if (MySQL_connect_flag == 1)
            {
                string AOI_Avg_Data = "INSERT INTO `aoi_avg_data_radius` (`serial_number`, `serial_id`, `Circle_Fitting`, `Ellipse_Fitting_a`, `Ellipse_Fitting_b`, `MaxInscribedCircle`, `MinEnclosingCircle`, `AreaToCircle`, `update_time`) VALUES (NULL, " + "'" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "','" + TB_Hough_Avg.Text + "','" + TB_EllipseA_Avg.Text + "','" + TB_EllipseB_Avg.Text + "','" + TB_Inner_Avg.Text + "','" + TB_Enclosing_Avg.Text + "','" + TB_Area_Avg.Text + "'" + ", CURRENT_TIMESTAMP);";
                string AOI_Max_Data = "INSERT INTO `aoi_max_data_radius` (`serial_number`, `serial_id`,`Circle_Fitting`, `Ellipse_Fitting_a`, `Ellipse_Fitting_b`, `MaxInscribedCircle`, `MinEnclosingCircle`, `AreaToCircle`, `update_time`) VALUES (NULL, " + "'" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "','" + TB_Hough_Max.Text + "','" + TB_EllipseA_Max.Text + "','" + TB_EllipseB_Max.Text + "','" + TB_Inner_Max.Text + "','" + TB_Enclosing_Max.Text + "','" + TB_Area_Max.Text + "'" + ", CURRENT_TIMESTAMP);";
                string AOI_Min_Data = "INSERT INTO `aoi_min_data_radius` (`serial_number`, `serial_id`, `Circle_Fitting`, `Ellipse_Fitting_a`, `Ellipse_Fitting_b`, `MaxInscribedCircle`, `MinEnclosingCircle`, `AreaToCircle`, `update_time`) VALUES (NULL, " + "'" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "','" + TB_Hough_Min.Text + "','" + TB_EllipseA_Min.Text + "','" + TB_EllipseB_Min.Text + "','" + TB_Inner_Min.Text + "','" + TB_Enclosing_Min.Text + "','" + TB_Area_Min.Text + "'" + ", CURRENT_TIMESTAMP);";
                string AOI_SD_Data = "INSERT INTO `aoi_sd_data_radius` (`serial_number`, `serial_id`,`Circle_Fitting`, `Ellipse_Fitting_a`, `Ellipse_Fitting_b`, `MaxInscribedCircle`, `MinEnclosingCircle`, `AreaToCircle`, `update_time`) VALUES (NULL, " + "'" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "','" + TB_Hough_SD.Text + "','" + TB_EllipseA_SD.Text + "','" + TB_EllipseB_SD.Text + "','" + TB_Inner_SD.Text + "','" + TB_Enclosing_SD.Text + "','" + TB_Area_SD.Text + "'" + ", CURRENT_TIMESTAMP);";

                string Realtime_AOI_Avg_Data = "UPDATE `aoi_realtime_data_radius` SET `Circle_Fitting`='" + TB_Hough_Avg.Text + "',`Ellipse_Fitting_a`='" + TB_EllipseA_Avg.Text + "',`Ellipse_Fitting_b`='" + TB_EllipseB_Avg.Text + "',`MaxInscribedCircle`='" + TB_Inner_Avg.Text + "',`MinEnclosingCircle`='" + TB_Enclosing_Avg.Text + "',`AreaToCircle`='" + TB_Area_Avg.Text + "',`serial_id`='" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "' WHERE `serial_number`='1'";
                string Realtime_AOI_SD_Data = "UPDATE `aoi_realtime_data_radius` SET `Circle_Fitting`='" + TB_Hough_SD.Text + "',`Ellipse_Fitting_a`='" + TB_EllipseA_SD.Text + "',`Ellipse_Fitting_b`='" + TB_EllipseB_SD.Text + "',`MaxInscribedCircle`='" + TB_Inner_SD.Text + "',`MinEnclosingCircle`='" + TB_Enclosing_SD.Text + "',`AreaToCircle`='" + TB_Area_SD.Text + "',`serial_id`='" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "' WHERE `serial_number`='2'";
                string Realtime_AOI_Max_Data = "UPDATE `aoi_realtime_data_radius` SET `Circle_Fitting`='" + TB_Hough_Max.Text + "',`Ellipse_Fitting_a`='" + TB_EllipseA_Max.Text + "',`Ellipse_Fitting_b`='" + TB_EllipseB_Max.Text + "',`MaxInscribedCircle`='" + TB_Inner_Max.Text + "',`MinEnclosingCircle`='" + TB_Enclosing_Max.Text + "',`AreaToCircle`='" + TB_Area_Max.Text + "',`serial_id`='" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "' WHERE `serial_number`='3'";
                string Realtime_AOI_Min_Data = "UPDATE `aoi_realtime_data_radius` SET `Circle_Fitting`='" + TB_Hough_Min.Text + "',`Ellipse_Fitting_a`='" + TB_EllipseA_Min.Text + "',`Ellipse_Fitting_b`='" + TB_EllipseB_Min.Text + "',`MaxInscribedCircle`='" + TB_Inner_Min.Text + "',`MinEnclosingCircle`='" + TB_Enclosing_Min.Text + "',`AreaToCircle`='" + TB_Area_Min.Text + "',`serial_id`='" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "' WHERE `serial_number`='4'";


                try
                {
                    connection.Open();
                    MySqlCommand AOI_Avg = new MySqlCommand(AOI_Avg_Data);
                    AOI_Avg.Connection = connection;
                    AOI_Avg.ExecuteNonQuery();

                    MySqlCommand AOI_Max = new MySqlCommand(AOI_Max_Data);
                    AOI_Max.Connection = connection;
                    AOI_Max.ExecuteNonQuery();

                    MySqlCommand AOI_Min = new MySqlCommand(AOI_Min_Data);
                    AOI_Min.Connection = connection;
                    AOI_Min.ExecuteNonQuery();

                    MySqlCommand AOI_SD = new MySqlCommand(AOI_SD_Data);
                    AOI_SD.Connection = connection;
                    AOI_SD.ExecuteNonQuery();

                    MySqlCommand R_AOI_Avg = new MySqlCommand(Realtime_AOI_Avg_Data);
                    R_AOI_Avg.Connection = connection;
                    R_AOI_Avg.ExecuteNonQuery();

                    MySqlCommand R_AOI_SD = new MySqlCommand(Realtime_AOI_SD_Data);
                    R_AOI_SD.Connection = connection;
                    R_AOI_SD.ExecuteNonQuery();

                    MySqlCommand R_AOI_Max = new MySqlCommand(Realtime_AOI_Max_Data);
                    R_AOI_Max.Connection = connection;
                    R_AOI_Max.ExecuteNonQuery();

                    MySqlCommand R_AOI_Min = new MySqlCommand(Realtime_AOI_Min_Data);
                    R_AOI_Min.Connection = connection;
                    R_AOI_Min.ExecuteNonQuery();
                    R_AOI_Min.Connection.Close();
                    connection.Close();

                    Item_ID1++;
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                    connection.Close();
                }
            }
            GC.Collect();
        }

        public float SD_Cal(float[] input_data, int num)
        {
            float SD_SUM = 0, SD = 0, SD_MEAN = 0;
            for (int i = 0; i < num; i++)
            {
                SD_SUM += input_data[i];
            }
            SD_MEAN = SD_SUM / num;
            for (int i = 0; i < num; i++)
            {
                SD += (float)(Math.Pow((input_data[i] - SD_MEAN), 2));
            }
            SD = (float)(Math.Sqrt(SD / num));
            return SD;
        }

        /**************** 還會再改 **********************/

        public void Autofocus()
        {
            Image<Gray, byte> Img_G1 = null;
            Image<Gray, byte> Img_G2 = null;
            Image<Gray, byte> Img_G3 = null;
            Image<Gray, float> Dft_Img_G3 = null;

            double prev = 0.0, Var = 0.0, max_Var = 0, max_Z = 0, Limit_Var = 200;
            double dz = 0.3;
            double X, Y, Z;
            Rectangle rect = new Rectangle();
            double u = new double();
            double AF_width = 0, AF_height = 0;
            float contour_ratio = 0;
            int error_count = 0;
            int AF_Change_count = 0;

            int pic_count = 1;
            int hole_count = 0;
            while (true)
            {
                //if (CCDTri_flag == 0)
                //{
                //    Get_CCDImg();
                //}
                //Img_G1 = CCD_Cap_Img;
                sw.Reset();
                sw.Start();
                hole_count = 0;
                if (pic_count > 10)
                    break;
                Img_G1 = new Image<Gray, byte>(pic_count.ToString() + ".bmp");
                Img_G2 = Img_G1;
                Img_G2 = Img_G2.Dilate(9);
                Img_G2 = Img_G2.Erode(9);
                Img_G2 = Img_G2.SmoothGaussian(21);
                Img_G2 = Img_G2.ThresholdBinaryInv(new Gray(17), new Gray(255)); //20
                Img_G2 = Img_G2.SmoothMedian(15);

                Contour<Point> contours = Img_G2.FindContours(
                Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_NONE,
                Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_EXTERNAL);

                for (; contours != null; contours = contours.HNext)
                {
                    contour_ratio = contours.GetMinAreaRect().size.Height / contours.GetMinAreaRect().size.Width;

                    if ((contours.Area >= 10000) && (contours.Area <= 200000) && (contour_ratio < 1.5) && (contour_ratio > 0.6))
                    {
                        hole_count++;
                        if (hole_count != 1)
                            continue;
                        rect = new Rectangle(new Point(contours.BoundingRectangle.X, contours.BoundingRectangle.Y), new Size(contours.BoundingRectangle.Width, contours.BoundingRectangle.Height));
                        Img_G3 = Img_G1.Copy(rect);

                        //FFT Method
                        Dft_Img_G3 = Img_G1.Copy(rect).Convert<Gray, float>();
                        IntPtr complexImage = CvInvoke.cvCreateImage(Dft_Img_G3.Size, Emgu.CV.CvEnum.IPL_DEPTH.IPL_DEPTH_32F, 2);
                        CvInvoke.cvSetZero(complexImage);
                        CvInvoke.cvSetImageCOI(complexImage, 1);
                        CvInvoke.cvCopy(Dft_Img_G3, complexImage, IntPtr.Zero);
                        CvInvoke.cvSetImageCOI(complexImage, 0);
                        Matrix<float> dft = new Matrix<float>(Dft_Img_G3.Rows, Dft_Img_G3.Cols, 2);
                        CvInvoke.cvDFT(complexImage, dft, Emgu.CV.CvEnum.CV_DXT.CV_DXT_FORWARD, 0);
                        Matrix<float> outReal = new Matrix<float>(Dft_Img_G3.Size);
                        Matrix<float> outIm = new Matrix<float>(Dft_Img_G3.Size);
                        CvInvoke.cvSplit(dft, outReal, outIm, IntPtr.Zero, IntPtr.Zero);
                        CvInvoke.cvShowImage("Real", outReal);
                        CvInvoke.cvShowImage("Imaginary ", outIm);

                        //Img_G3.Save(pic_count.ToString() + "_AF.bmp");
                        //u = Img_G3.GetAverage().Intensity;
                        AF_width = Img_G3.Width;
                        AF_height = Img_G3.Height;
                        Var = 0.0;
                        for (int i = 0; i < AF_width; i++)
                        {
                            for (int j = 0; j < AF_height; j++)
                            {
                                //Var += Math.Pow(Img_G3[j, i].Intensity - u, 2);
                                Var += Math.Sqrt(Math.Pow(outReal[j, i], 2) + (Math.Pow(outIm[j, i], 2) * Math.Abs(Math.Atan2(outIm[j, i], outReal[j, i]))));
                            }
                        }
                        Var /= (AF_width * AF_height);
                        //Console.WriteLine(Var.ToString());
                        break;
                    }
                }
                sw.Stop();

                Console.WriteLine(pic_count++.ToString() + " : " + Var.ToString() + ", Time : " + sw.ElapsedMilliseconds + "ms");
                //if (Var > max_Var)
                //{
                //    max_Var = Var;
                //    max_Z = Convert.ToDouble(TB_NM_Z.Text);
                //}
                //if (AF_Change_count >= 2)
                //{
                //    Limit_Var = max_Var;
                //    this.Invoke((MethodInvoker)delegate ()
                //    {
                //        TB_NM_Z.Text = max_Z.ToString();
                //    });
                //    while (TB_NM_Z.Text != max_Z.ToString()) ;
                //    NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), max_Z, Convert.ToDouble(TB_NM_Phi.Text), Convert.ToDouble(TB_NM_Theta.Text), Convert.ToDouble(TB_NM_Psi.Text));
                //    while (EDM_Param.EDM_Check) ;
                //    EDM_Param.EDM_Check = true;
                //    break;
                //}
                //if (Var - prev < -2.0)
                //{
                //    error_count++;
                //    this.Invoke((MethodInvoker)delegate ()
                //    {
                //        TB_Consoletext.Text = "Error Dir : " + error_count.ToString();
                //    });
                //    if (error_count == 2)
                //    {
                //        AF_Change_count++;
                //        error_count = 0;
                //        dz *= -1;
                //    }
                //}
                //else
                //{
                //    error_count = 0;
                //    this.Invoke((MethodInvoker)delegate ()
                //    {
                //        TB_Consoletext.Text = "Correct Dir.";
                //    });

                //}
                //prev = Var;
                //if (Var < Limit_Var)
                //    break;
                //X = Convert.ToDouble(TB_NM_X.Text);
                //Y = Convert.ToDouble(TB_NM_Y.Text);
                //Z = Convert.ToDouble(TB_NM_Z.Text) + dz;
                //this.Invoke((MethodInvoker)delegate ()
                //{
                //    TB_NM_Z.Text = Z.ToString();
                //});
                //while (TB_NM_Z.Text != Z.ToString()) ;
                //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), Z, Convert.ToDouble(TB_NM_Phi.Text), Convert.ToDouble(TB_NM_Theta.Text), Convert.ToDouble(TB_NM_Psi.Text));
                //while (EDM_Param.EDM_Check) ;
                //EDM_Param.EDM_Check = true;
                //Thread.Sleep(100);
                GC.Collect();
            }
            this.Invoke((MethodInvoker)delegate ()
            {
                TB_Consoletext.Text = "Auto Focus Completed.";
            });
        }

        //public void AOI_First_Positioning()
        //{
        //    Image<Gray, byte> Img_G1 = null;
        //    Image<Gray, byte> Img_G2 = null;
        //    double X, Y, Z;
        //    double dx = 0.4;
        //    int hole_count = 0;
        //    float contour_ratio = 0;
        //    First_Hole_center.Y = 3036;
        //    while (true)
        //    {
        //        hole_count = 0;
        //        if (CCDTri_flag == 0)
        //        {
        //            Get_CCDImg();
        //        }
        //        Img_G1 = CCD_Cap_Img;
        //        Img_G2 = Img_G1;
        //        Img_G2 = Img_G2.SmoothGaussian(15);
        //        Img_G2 = Img_G2.ThresholdBinaryInv(new Gray(30), new Gray(255)); //20
        //        Img_G2 = Img_G2.SmoothMedian(15);
        //        Contour<Point> contours = Img_G2.FindContours(
        //        Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_NONE,
        //        Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_EXTERNAL);
        //        for (; contours != null; contours = contours.HNext)
        //        {
        //            contour_ratio = contours.GetMinAreaRect().size.Height / contours.GetMinAreaRect().size.Width;
        //            if ((contours.Area >= 10000) && (contours.Area <= 200000) && (contour_ratio < 1.5) && (contour_ratio > 0.6))
        //            {
        //                if (First_Hole_center.Y > contours.GetMinAreaRect().center.Y)
        //                {
        //                    First_Hole_center = contours.GetMinAreaRect().center;
        //                }
        //                hole_count++;
        //            }
        //        }
        //        if (hole_count > 12)
        //        {
        //            dx = -0.4;
        //        }
        //        else if (hole_count < 12)
        //        {
        //            dx = 0.4;
        //        }
        //        else
        //        {
        //            this.Invoke((MethodInvoker)delegate ()
        //            {
        //                TB_Consoletext.Text = "AOI First Positioning Completed.";
        //            });
        //            break;
        //        }
        //        X = Convert.ToDouble(TB_NM_X.Text) + dx;
        //        Y = Convert.ToDouble(TB_NM_Y.Text);
        //        Z = Convert.ToDouble(TB_NM_Z.Text);
        //        this.Invoke((MethodInvoker)delegate ()
        //        {
        //            TB_NM_X.Text = X.ToString();
        //        });
        //        while (TB_NM_X.Text != X.ToString()) ;
        //        NM_Move(X, Y, Z, Convert.ToDouble(TB_NM_Phi.Text), Convert.ToDouble(TB_NM_Theta.Text), Convert.ToDouble(TB_NM_Psi.Text));
        //        Thread.Sleep(100);
        //    }
        //}

        private void First_Hole_Affine()
        {
            double u = new double();
            double v = new double();

            double ori_X = 1600;
            double ori_Y = 192.35;
            double X = 0;
            double Y = 0;
            v = (First_Hole_center.X - ori_X) / 493.22;
            u = (First_Hole_center.Y - ori_Y) / 264.43;

            this.Invoke((MethodInvoker)delegate ()
            {
                TB_Consoletext.Text = "First Hole Affine Completed.";
                X = Convert.ToDouble(TB_NM_X.Text) + u;
                Y = Convert.ToDouble(TB_NM_Y.Text) + v;
                TB_NM_X.Text = X.ToString();
                TB_NM_Y.Text = Y.ToString();
            });
            while (TB_NM_Y.Text != Y.ToString()) ;
        }

        //private void Affine1()
        //{
        //    double u = new double();
        //    double v = new double();
        //    Matrix<float> mat = new Matrix<float>(2, 3);
        //    PointF[] srcTri = new PointF[3];
        //    PointF[] dstTri = new PointF[3];

        //    srcTri[0] = new PointF(632, 455);
        //    srcTri[1] = new PointF(873, 301);
        //    srcTri[2] = new PointF(376, 312);

        //    dstTri[0] = new PointF(0, 0);
        //    dstTri[1] = new PointF(100, 62);
        //    dstTri[2] = new PointF(-100, 62);

        //    ////Draw the corners
        //    CvInvoke.cvGetAffineTransform(srcTri, dstTri, mat);

        //    u = Math.Round((Convert.ToInt16(P1_X) * mat[0, 0] + Convert.ToInt16(P1_Y) * mat[0, 1] + mat[0, 2]), 4, MidpointRounding.AwayFromZero);
        //    v = Math.Round((Convert.ToInt16(P1_X) * mat[1, 0] + Convert.ToInt16(P1_Y) * mat[1, 1] + mat[1, 2]), 4, MidpointRounding.AwayFromZero);
        //    Console.WriteLine(u.ToString() + "," + v.ToString() + "\n");


        //    J6_rotate_angle = Convert.ToDouble(TB_NM_Psi.Text) + 90;

        //    this.Invoke((MethodInvoker)delegate ()
        //    {
        //        TB_NM_X.Text = (Convert.ToDouble(TB_NM_X.Text) + u * Math.Cos(J6_rotate_angle * Math.PI / 180) + v * Math.Sin(J6_rotate_angle * Math.PI / 180)).ToString();
        //        TB_NM_Y.Text = (Convert.ToDouble(TB_NM_Y.Text) - u * Math.Sin(J6_rotate_angle * Math.PI / 180) + v * Math.Cos(J6_rotate_angle * Math.PI / 180)).ToString();
        //    });

        //    for (int i = 0; i < mat.Rows; i++)
        //    {
        //        for (int j = 0; j < mat.Cols; j++)
        //            Console.Write(mat[i, j] + "\t");
        //        Console.WriteLine(" ");
        //    }
        //}

        //private void Affine2()
        //{
        //    double u = new double();
        //    double v = new double();
        //    //double ori_X = 673;
        //    //double ori_Y = 523;
        //    double ori_X = 646;
        //    double ori_Y = 528;
        //    u = (P1_X - ori_X) / 10;
        //    v = -(P1_Y - ori_Y) / 3;

        //    J6_rotate_angle = Convert.ToDouble(TB_NM_Psi.Text) + 90;


        //    Console.WriteLine(u.ToString() + "," + v.ToString() + "\n");

        //    this.Invoke((MethodInvoker)delegate ()
        //    {
        //        TB_NM_X.Text = (Convert.ToDouble(TB_NM_X.Text) + u * Math.Cos(J6_rotate_angle * Math.PI / 180) + v * Math.Sin(J6_rotate_angle * Math.PI / 180)).ToString();
        //        TB_NM_Y.Text = (Convert.ToDouble(TB_NM_Y.Text) - u * Math.Sin(J6_rotate_angle * Math.PI / 180) + v * Math.Cos(J6_rotate_angle * Math.PI / 180)).ToString();
        //    });
        //}

        public delegate void UIupdate();
        private void AOI_Task()
        {
            count = 0;
            count_fit = 0;
            double[] hole_Z = new double[10] { 295.12, 296.12, 297.12, 298.12, 299.12, 300.12,
                                               299.12, 300.12, 301.12, 302.12};
            bool Position_Check = false;
            this.Invoke((MethodInvoker)delegate ()
            {
                RTB_NC_Info.Text = "Robot Position : \n";
            });
            for (int i = 0; i < NC_Obj_P.counter-1; i++)
            {
                this.Invoke((MethodInvoker)delegate ()
                {
                    if(i > 5)
                    {
                        TB_NM_X.Text = "-2.678";
                        TB_NM_Y.Text = "784.27"; 
                    }
                    else
                    {
                        TB_NM_X.Text = "-2.678";
                        TB_NM_Y.Text = "774.27";
                    }
                    TB_NM_Z.Text = hole_Z[i].ToString();

                    //TB_NM_X.Text = (Convert.ToDouble(TB_NM_X.Text) + NC_Obj_P.X[i]).ToString();
                    //TB_NM_Y.Text = (Convert.ToDouble(TB_NM_Y.Text) + NC_Obj_P.Y[i]).ToString();
                    //TB_NM_Z.Text = (Convert.ToDouble(TB_NM_Z.Text) + NC_Obj_P.Z[i]).ToString();
                    //TB_NM_Phi.Text = ((-180 + NC_Obj_P.A[i]) % 360).ToString();
                    //TB_NM_Psi.Text = ((-90 + NC_Obj_P.C[i]) % 360).ToString();
                    RTB_NC_Info.Text += "X : " + TB_NM_X.Text + ", Y : " + TB_NM_Y.Text + ", Z : " + TB_NM_Z.Text + "\nPhi : " + TB_NM_Phi.Text + ", Theta : " + TB_NM_Theta.Text + ", Psi : " + TB_NM_Psi.Text + "\n";
                    RTB_NC_Info.Text += "-----------------------------------------------------\n";
                    Position_Check = true;
                });
                while (!Position_Check) ;
                //Position_Check = false; 
                //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), Convert.ToDouble(TB_NM_Z.Text), Convert.ToDouble(TB_NM_Phi.Text) * Math.PI / 180, Convert.ToDouble(TB_NM_Theta.Text) * Math.PI / 180, Convert.ToDouble(TB_NM_Psi.Text) * Math.PI / 180);
                //while (EDM_Param.EDM_Check) ;
                //EDM_Param.EDM_Check = true;
                //Thread.Sleep(1000);
                AOI(i);

                //Autofocus();
                //First_Hole_Affine();
                //NM_Move(Convert.ToDouble(TB_NM_X.Text), Convert.ToDouble(TB_NM_Y.Text), Convert.ToDouble(TB_NM_Z.Text), Convert.ToDouble(TB_NM_Phi.Text)* Math.PI / 180, Convert.ToDouble(TB_NM_Theta.Text)* Math.PI / 180, Convert.ToDouble(TB_NM_Psi.Text)* Math.PI / 180);
                //while (EDM_Param.EDM_Check) ;
                //EDM_Param.EDM_Check = true;
                //Autofocus();
            }

            
            this.Invoke((MethodInvoker)delegate ()
            {
                UIupdate uiset = new UIupdate(AOI_Message);
                this.BeginInvoke(uiset);
                TB_Consoletext.Text = "AOI Completed.";
            });
        }

        private void AOI(int h_count)
        {
            Image<Rgb, byte> Img_C1 = null;
            Image<Rgb, byte> Img_C2 = null;
            Image<Gray, byte> Img_G1 = null;
            Image<Gray, byte> Img_G2 = null;
            Image<Gray, byte> Img_G3 = null;
            Image<Gray, byte> Img_G4 = null;
            int hole_P = 0;
            if (CCDTri_flag == 0)
            {
                Get_CCDImg();
            }
            //Img_G1 = new Image<Gray, byte>((h_count+1).ToString() + ".bmp");
            Img_G1 = CCD_Cap_Img;
            Img_G2 = Img_G1;
            Img_G3 = Img_G1.CopyBlank();
            Img_G4 = Img_G1.CopyBlank();
            Img_C1 = Img_G1.Convert<Rgb, byte>();
            Img_C2 = Img_G1.Convert<Rgb, byte>();

            Img_G2 = Img_G2.Dilate(9);
            Img_G2 = Img_G2.Erode(9);
            Img_G2 = Img_G2.SmoothGaussian(21);
            Img_G2 = Img_G2.ThresholdBinaryInv(new Gray(17), new Gray(255)); //20
            Img_G2 = Img_G2.SmoothMedian(15);
            Contour<Point> Fit_hole = null;
            Contour<Point> contours = Img_G2.FindContours(
            Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_NONE,
            Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_EXTERNAL);
            MCvFont font = new MCvFont();
            CvInvoke.cvInitFont(ref font, Emgu.CV.CvEnum.FONT.CV_FONT_HERSHEY_COMPLEX, 3, 3, 0, 4, Emgu.CV.CvEnum.LINE_TYPE.FOUR_CONNECTED);
            for (; contours != null; contours = contours.HNext)
            {
                contour_ratio = contours.GetMinAreaRect().size.Height / contours.GetMinAreaRect().size.Width;
                if ((contours.Area >= 10000) && (contours.Area <= 200000) && (contour_ratio < 2) && (contour_ratio > 0.5))
                {
                    if (hole_P != (h_count%6))
                    {
                        hole_P++;
                        continue;
                    }    
                    else
                    {
                        Fit_hole = contours;
                        First_Hole_center = contours.GetMinAreaRect().center;
                        contour_perimeter[count] = (float)(contours.Perimeter * 3.7); //2.36174
                        contour_perimeter[count] = (float)(CvInvoke.cvArcLength(contours, MCvSlice.WholeSeq, -1) * 3.7);

                        Area_diameter[count] = (float)(Math.Sqrt(contours.Area / Math.PI) * 2 * 3.7);
                        Img_C1.Draw(contours, new Rgb(Color.White), 1);
                        //Img_C1.Draw((First_Hole_center.X).ToString() + "," + (First_Hole_center.Y).ToString(), ref font, new Point((int)contours.GetMinAreaRect().center.X, (int)contours.GetMinAreaRect().center.Y), new Rgb(Color.Red));
                        Img_C2.Draw(contours, new Rgb(Color.White), 2);
                        Img_G3.Draw(contours, new Gray(255), 1);

                        #region 內切圓
                        double dist = 0;
                        double maxdist = 0;
                        PointF center = new PointF(0, 0);
                        float X_left = (float)(contours.BoundingRectangle.X + contours.BoundingRectangle.Width * 0.25);
                        float X_right = (float)(contours.BoundingRectangle.X + contours.BoundingRectangle.Width * 0.75);
                        float Y_top = (float)(contours.BoundingRectangle.Y + contours.BoundingRectangle.Height * 0.25);
                        float Y_bottom = (float)(contours.BoundingRectangle.Y + contours.BoundingRectangle.Height * 0.75);

                        for (float k = X_left; k < X_right; k = k + 5)
                        {
                            for (float j = Y_top; j < Y_bottom; j = j + 5)
                            {
                                PointF pt = new PointF(k, j);
                                //ptc[0] = Point.Round(pt);
                                //newimage1.DrawPolyline(ptc, true, new Rgb(Color.Yellow), 2);
                                dist = CvInvoke.cvPointPolygonTest(contours, pt, true); //求得pt與輪廓的距離 內部正值,外部負值,線上0
                                if (dist > maxdist)
                                {
                                    maxdist = dist;
                                    center = pt;
                                }
                            }
                        }
                        inner_center[count] = center;
                        inner_diameter[count] = (float)maxdist;
                        Img_C1.Draw(new CircleF(inner_center[count], (float)inner_diameter[count]), new Rgb(Color.Red), 2);
                        #endregion

                        #region 外接圓
                        CvInvoke.cvMinEnclosingCircle(contours, out smallest_center[count], out smallest_diameter[count]);
                        Img_C1.Draw(new CircleF(smallest_center[count], (float)smallest_diameter[count]), new Rgb(Color.Green), 2);
                        #endregion

                        #region 橢圓擬合
                        Ellipse fittedEllipse = new Ellipse(CvInvoke.cvFitEllipse2(contours));
                        fit_ellipse_center[count] = fittedEllipse.MCvBox2D.center;
                        fit_ellipse_diameter1[count] = fittedEllipse.MCvBox2D.size.Height;
                        fit_ellipse_diameter2[count] = fittedEllipse.MCvBox2D.size.Width;
                        fit_ellipse_angle[count] = fittedEllipse.MCvBox2D.angle;
                        Img_C1.Draw(new Ellipse(fit_ellipse_center[count], new SizeF(fit_ellipse_diameter1[count], fit_ellipse_diameter2[count]), fit_ellipse_angle[count]), new Rgb(Color.SkyBlue), 2);
                        #endregion

                        count++;
                        break;
                    } 
                }
            }
            Img_G3.Save("G3.bmp");
            #region 霍夫找圓
            CircleF[][] fit_circles = Img_G3.HoughCircles(      //霍夫找圓
            new Gray(255), //Canny algorithm high threshold
                           //(the lower one will be twice smaller)
            new Gray(100),  //accumulator threshold at the center detection stage
            4,             //accumulator resolution
            200,           //偵測到任意兩圓心最小距離
            65,           //最小半徑
            85);         //最大半徑

            foreach (CircleF circle in fit_circles[0])
            {
                fit_circle_center[count_fit] = circle.Center;
                fit_circle_diameter[count_fit] = (float)(circle.Radius * 2 * 3.7);
                Img_C1.Draw(circle, new Rgb(Color.Orange), 2);
                count_fit++;                    
            }
            #endregion

            IB_1.Image = Img_G1;
            IB_2.Image = Img_C1;
            Img_C1.ToBitmap().Save(count.ToString() + "_B.bmp");
            //Img_C1.Save(count.ToString() + "_C.bmp");

            GC.Collect();
        }

        private void AOI_Message()
        {
            for (int i = 0; i < count; i++)
            {
                inner_diameter[i] = (float)(inner_diameter[i] * 2 * 3.7);
                smallest_diameter[i] = (float)(smallest_diameter[i] * 2 * 3.7);
                fit_ellipse_diameter1[i] = (float)(fit_ellipse_diameter1[i] * 3.7);
                fit_ellipse_diameter2[i] += (float)(fit_ellipse_diameter2[i] * 3.7);
            }

            #region 標準差
            TB_Hough_SD.Text = SD_Cal(fit_circle_diameter, count_fit).ToString();
            TB_Area_SD.Text = SD_Cal(Area_diameter, count).ToString();
            TB_Enclosing_SD.Text = SD_Cal(smallest_diameter, count).ToString();
            TB_Inner_SD.Text = SD_Cal(inner_diameter, count).ToString();
            TB_EllipseA_SD.Text = SD_Cal(fit_ellipse_diameter1, count).ToString();
            TB_EllipseB_SD.Text = SD_Cal(fit_ellipse_diameter2, count).ToString();
            TB_Perimeter_SD.Text = SD_Cal(contour_perimeter, count).ToString();
            #endregion

            #region 最小值
            TB_Hough_Min.Text = fit_circle_diameter.Where(q => q != 0).Min().ToString();
            TB_Area_Min.Text = Area_diameter.Where(q => q != 0).Min().ToString();
            TB_Enclosing_Min.Text = smallest_diameter.Where(q => q != 0).Min().ToString();
            TB_Inner_Min.Text = inner_diameter.Where(q => q != 0).Min().ToString();
            TB_EllipseA_Min.Text = fit_ellipse_diameter1.Where(q => q != 0).Min().ToString();
            TB_EllipseB_Min.Text = fit_ellipse_diameter2.Where(q => q != 0).Min().ToString();
            TB_Perimeter_Min.Text = contour_perimeter.Where(q => q != 0).Min().ToString();
            #endregion

            #region 最大值
            TB_Hough_Max.Text = fit_circle_diameter.Max().ToString();
            TB_Area_Max.Text = Area_diameter.Max().ToString();
            TB_Enclosing_Max.Text = smallest_diameter.Max().ToString();
            TB_Inner_Max.Text = inner_diameter.Max().ToString();
            TB_EllipseA_Max.Text = fit_ellipse_diameter1.Max().ToString();
            TB_EllipseB_Max.Text = fit_ellipse_diameter2.Max().ToString();
            TB_Perimeter_Max.Text = contour_perimeter.Max().ToString();
            #endregion

            #region 平均值
            TB_Hough_Avg.Text = (fit_circle_diameter.Sum() / count_fit).ToString();
            TB_Area_Avg.Text = (Area_diameter.Sum() / count).ToString();
            TB_Enclosing_Avg.Text = (smallest_diameter.Sum() / count).ToString();
            TB_Inner_Avg.Text = (inner_diameter.Sum() / count).ToString();
            TB_EllipseA_Avg.Text = (fit_ellipse_diameter1.Sum() / count).ToString();
            TB_EllipseB_Avg.Text = (fit_ellipse_diameter2.Sum() / count).ToString();
            TB_Perimeter_Avg.Text = (contour_perimeter.Sum() / count).ToString();
            #endregion

            count = 0;
            count_fit = 0;

            if (MySQL_connect_flag == 1)
            {
                string AOI_Avg_Data = "INSERT INTO `aoi_avg_data_radius` (`serial_number`, `serial_id`, `Circle_Fitting`, `Ellipse_Fitting_a`, `Ellipse_Fitting_b`, `MaxInscribedCircle`, `MinEnclosingCircle`, `AreaToCircle`, `update_time`) VALUES (NULL, " + "'" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "','" + TB_Hough_Avg.Text + "','" + TB_EllipseA_Avg.Text + "','" + TB_EllipseB_Avg.Text + "','" + TB_Inner_Avg.Text + "','" + TB_Enclosing_Avg.Text + "','" + TB_Area_Avg.Text + "'" + ", CURRENT_TIMESTAMP);";
                string AOI_Max_Data = "INSERT INTO `aoi_max_data_radius` (`serial_number`, `serial_id`,`Circle_Fitting`, `Ellipse_Fitting_a`, `Ellipse_Fitting_b`, `MaxInscribedCircle`, `MinEnclosingCircle`, `AreaToCircle`, `update_time`) VALUES (NULL, " + "'" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "','" + TB_Hough_Max.Text + "','" + TB_EllipseA_Max.Text + "','" + TB_EllipseB_Max.Text + "','" + TB_Inner_Max.Text + "','" + TB_Enclosing_Max.Text + "','" + TB_Area_Max.Text + "'" + ", CURRENT_TIMESTAMP);";
                string AOI_Min_Data = "INSERT INTO `aoi_min_data_radius` (`serial_number`, `serial_id`, `Circle_Fitting`, `Ellipse_Fitting_a`, `Ellipse_Fitting_b`, `MaxInscribedCircle`, `MinEnclosingCircle`, `AreaToCircle`, `update_time`) VALUES (NULL, " + "'" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "','" + TB_Hough_Min.Text + "','" + TB_EllipseA_Min.Text + "','" + TB_EllipseB_Min.Text + "','" + TB_Inner_Min.Text + "','" + TB_Enclosing_Min.Text + "','" + TB_Area_Min.Text + "'" + ", CURRENT_TIMESTAMP);";
                string AOI_SD_Data = "INSERT INTO `aoi_sd_data_radius` (`serial_number`, `serial_id`,`Circle_Fitting`, `Ellipse_Fitting_a`, `Ellipse_Fitting_b`, `MaxInscribedCircle`, `MinEnclosingCircle`, `AreaToCircle`, `update_time`) VALUES (NULL, " + "'" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "','" + TB_Hough_SD.Text + "','" + TB_EllipseA_SD.Text + "','" + TB_EllipseB_SD.Text + "','" + TB_Inner_SD.Text + "','" + TB_Enclosing_SD.Text + "','" + TB_Area_SD.Text + "'" + ", CURRENT_TIMESTAMP);";

                string Realtime_AOI_Avg_Data = "UPDATE `aoi_realtime_data_radius` SET `Circle_Fitting`='" + TB_Hough_Avg.Text + "',`Ellipse_Fitting_a`='" + TB_EllipseA_Avg.Text + "',`Ellipse_Fitting_b`='" + TB_EllipseB_Avg.Text + "',`MaxInscribedCircle`='" + TB_Inner_Avg.Text + "',`MinEnclosingCircle`='" + TB_Enclosing_Avg.Text + "',`AreaToCircle`='" + TB_Area_Avg.Text + "',`serial_id`='" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "' WHERE `serial_number`='1'";
                string Realtime_AOI_SD_Data = "UPDATE `aoi_realtime_data_radius` SET `Circle_Fitting`='" + TB_Hough_SD.Text + "',`Ellipse_Fitting_a`='" + TB_EllipseA_SD.Text + "',`Ellipse_Fitting_b`='" + TB_EllipseB_SD.Text + "',`MaxInscribedCircle`='" + TB_Inner_SD.Text + "',`MinEnclosingCircle`='" + TB_Enclosing_SD.Text + "',`AreaToCircle`='" + TB_Area_SD.Text + "',`serial_id`='" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "' WHERE `serial_number`='2'";
                string Realtime_AOI_Max_Data = "UPDATE `aoi_realtime_data_radius` SET `Circle_Fitting`='" + TB_Hough_Max.Text + "',`Ellipse_Fitting_a`='" + TB_EllipseA_Max.Text + "',`Ellipse_Fitting_b`='" + TB_EllipseB_Max.Text + "',`MaxInscribedCircle`='" + TB_Inner_Max.Text + "',`MinEnclosingCircle`='" + TB_Enclosing_Max.Text + "',`AreaToCircle`='" + TB_Area_Max.Text + "',`serial_id`='" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "' WHERE `serial_number`='3'";
                string Realtime_AOI_Min_Data = "UPDATE `aoi_realtime_data_radius` SET `Circle_Fitting`='" + TB_Hough_Min.Text + "',`Ellipse_Fitting_a`='" + TB_EllipseA_Min.Text + "',`Ellipse_Fitting_b`='" + TB_EllipseB_Min.Text + "',`MaxInscribedCircle`='" + TB_Inner_Min.Text + "',`MinEnclosingCircle`='" + TB_Enclosing_Min.Text + "',`AreaToCircle`='" + TB_Area_Min.Text + "',`serial_id`='" + Item_ID0.ToString() + "_" + Item_ID1.ToString() + "' WHERE `serial_number`='4'";


                try
                {
                    connection.Open();
                    MySqlCommand AOI_Avg = new MySqlCommand(AOI_Avg_Data);
                    AOI_Avg.Connection = connection;
                    AOI_Avg.ExecuteNonQuery();

                    MySqlCommand AOI_Max = new MySqlCommand(AOI_Max_Data);
                    AOI_Max.Connection = connection;
                    AOI_Max.ExecuteNonQuery();

                    MySqlCommand AOI_Min = new MySqlCommand(AOI_Min_Data);
                    AOI_Min.Connection = connection;
                    AOI_Min.ExecuteNonQuery();

                    MySqlCommand AOI_SD = new MySqlCommand(AOI_SD_Data);
                    AOI_SD.Connection = connection;
                    AOI_SD.ExecuteNonQuery();

                    MySqlCommand R_AOI_Avg = new MySqlCommand(Realtime_AOI_Avg_Data);
                    R_AOI_Avg.Connection = connection;
                    R_AOI_Avg.ExecuteNonQuery();

                    MySqlCommand R_AOI_SD = new MySqlCommand(Realtime_AOI_SD_Data);
                    R_AOI_SD.Connection = connection;
                    R_AOI_SD.ExecuteNonQuery();

                    MySqlCommand R_AOI_Max = new MySqlCommand(Realtime_AOI_Max_Data);
                    R_AOI_Max.Connection = connection;
                    R_AOI_Max.ExecuteNonQuery();

                    MySqlCommand R_AOI_Min = new MySqlCommand(Realtime_AOI_Min_Data);
                    R_AOI_Min.Connection = connection;
                    R_AOI_Min.ExecuteNonQuery();
                    R_AOI_Min.Connection.Close();
                    connection.Close();

                    Item_ID1++;
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                    connection.Close();
                }
            }

            GC.Collect();
        }
        /************************************************/

        #endregion

        private void BT_Mode_Click(object sender, EventArgs e)
        {
            JOG_flag ^= 1;
            if (JOG_flag == 1)
            {
                GB_NM_Move.Enabled = false;
                GB_EDM.Enabled = false;
                GB_JOG.Enabled = true;
                for (int axis = 0; axis < 6; axis++)
                    tcClient.WriteAny(Group, SlaveRegister[3] + axis * devicelength, ModeCSV);
                LB_Mode.Text = "Mode : ModeCSV";
            }
            else
            {
                GB_NM_Move.Enabled = true;
                GB_EDM.Enabled = true;
                GB_JOG.Enabled = false;
                AdsReadFeedBack();
                AdsWriteRead();
                for (int Axis = 0; Axis < 6; Axis++)
                {
                    axiswrite[Axis].ModesOfOperation = ModeCSP;
                    axiswrite[Axis].ControlWord = 0x0f;
                    axiswrite[Axis].TargetPosition = axisfeedback[Axis].Position;
                }
                AdsWrite();
                LB_Mode.Text = "Mode : ModeCSP";
            }
        }

        private void BT_Grip_Open_Click(object sender, EventArgs e)
        {
            EG_Control_API.RunMove(32, 40);
        }

        private void BT_Grip_Close_Click(object sender, EventArgs e)
        {
            //EG_Control_API.RunExpert('C', 29, 10, 0, 20, 40);    //'C'=Close/'O'=Open, 相對位置, 夾取速度, 夾取時位移, 夾取時位移速度, 夾取力量大小
            EG_Control_API.RunMove(7, 40);
        }
    }

    public class ShareArea
    {
        //Grip comport
        public static int Gripport = 0;
    }
}
