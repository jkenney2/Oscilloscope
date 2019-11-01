using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using System.ComponentModel;
using System.Threading;
using Microsoft.Win32;
using System.IO;


namespace Oscilloscope
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {

        //Oscilloscope Host Setup Constants
        private readonly OscilloscopeHost host = null;
        private readonly double HostWidth = 800.0;
        private readonly double HostHeight = 500.0;
        private readonly Brush BackgroundBrush = Brushes.LightGray;
        private readonly Brush HorizontalGridBrush = Brushes.Black;
        private readonly Brush VerticalGridBrush = Brushes.Black;
        private readonly double MajorGridThickness = 1.0;
        private readonly double MinorGridThickness = 0.5;
        private readonly double MajorVerticalGridResolution = 4.0;
        private readonly double MinorVerticalTicksPerMajor = 5.0;
        private readonly double MajorHorizontalGridResolution = 5.0;
        private readonly double MinorHorizontalGridResolution = 25.0;

        //For Drawing Oscilloscope Data
        private readonly Pen OscilloscopePen = new Pen(Brushes.Red, 2.0);

        //Frequency Mode
        //FrequencyMode freqMode = FrequencyMode.Normal;
        //bool freqModeChanged = false;

        //Maximum 14 bit unsigned number for updating base time -- for use in DoWork3
        private readonly ulong max14Bit = 16384UL;

        //Microseconds between data acquisitions
        private readonly ulong acqTime = 25L;

        //Variables for Display Scaling
        private double timeScale;
        private double voltScale;
        private double voltMin;
        private double minorVerticalGridResolution;
        private object locker = new object();

        //List of Stored Oscilloscope Data, and indices pointing to the data
        private List<OscilloscopeData> data = new List<OscilloscopeData>();
        private int dataReadRatio = 1;
        //private int triggerTargetCount = 0;
        private double triggerVolts = 2.5;
        private bool triggerOn = false;

        //BackgroundWorker for asynchrounous serial communication
        BackgroundWorker backgroundWorker;

        //Parameters for serial communication
        private int baudRate = 460800;
        private readonly Parity Parity = Parity.None;
        private readonly int DataBits = 8;
        private readonly StopBits StopBits = StopBits.One;

        //Interval for Reporting Progress
        private readonly TimeSpan ProgressReportingInterval = TimeSpan.FromSeconds(0.02);

        //PointListPair to receive data when backgroundWorker has completed
        List<OscilloscopeData> returnedData = null;

        public MainWindow()
        {
            InitializeComponent();
            host = new OscilloscopeHost();
            Grid.SetRow(host, 1);
            Grid.SetColumn(host, 1);
            MainGrid.Children.Add(host);
            host.Height = HostHeight;
            host.Width = HostWidth;

            backgroundWorker = new BackgroundWorker();
            backgroundWorker.WorkerSupportsCancellation = true;
            backgroundWorker.WorkerReportsProgress = true;
            backgroundWorker.DoWork += backgroundWorker_DoWork3;
            backgroundWorker.ProgressChanged += backgroundWorker_ProgressChanged;
            backgroundWorker.RunWorkerCompleted += backgroundWorker_RunWorkerCompleted;
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            SetupHost();
            string[] portNames = SerialPort.GetPortNames();
            if (portNames != null && portNames.Length > 0)
            {
                cbxPorts.ItemsSource = portNames;
                if (!string.IsNullOrEmpty(portNames[0])) cbxPorts.SelectedIndex = 0;
            }
            ui_Inputs_Changed(null, null);

        }

        private void SetupHost()
        {
            double MinorVerticalGridResolution = MajorVerticalGridResolution * MinorVerticalTicksPerMajor + 2.0;
            DrawingGroup hostDrawingGroup = new DrawingGroup();
            hostDrawingGroup.Children.Add(new GeometryDrawing(BackgroundBrush, null, new RectangleGeometry(new Rect(0.0, 0.0, HostWidth, HostHeight))));
            for (int i = 1; i < MinorHorizontalGridResolution; i++)
                hostDrawingGroup.Children.Add(new GeometryDrawing(null, new Pen(HorizontalGridBrush, MinorGridThickness),
                    new LineGeometry(new Point(i * HostWidth / MinorHorizontalGridResolution, 0), new Point(i * HostWidth / MinorHorizontalGridResolution, HostHeight))));
            for (int i = 1; i < MajorHorizontalGridResolution; i++)
                hostDrawingGroup.Children.Add(new GeometryDrawing(null, new Pen(HorizontalGridBrush, MajorGridThickness),
                    new LineGeometry(new Point(i * HostWidth / MajorHorizontalGridResolution, 0), new Point(i * HostWidth / MajorHorizontalGridResolution, HostHeight))));
            for (int i = 1; i < MinorVerticalGridResolution; i++)
                hostDrawingGroup.Children.Add(new GeometryDrawing(null, new Pen(VerticalGridBrush, MinorGridThickness),
                    new LineGeometry(new Point(0, i * HostHeight / MinorVerticalGridResolution), new Point(HostWidth, i * HostHeight / MinorVerticalGridResolution))));
            for (int i = 0; i <= MajorVerticalGridResolution; i++)
                hostDrawingGroup.Children.Add(new GeometryDrawing(null, new Pen(VerticalGridBrush, MajorGridThickness),
                    new LineGeometry(new Point(0, (1 + i * MinorVerticalTicksPerMajor) * HostHeight / MinorVerticalGridResolution),
                    new Point(HostWidth, (1 + i * MinorVerticalTicksPerMajor) * HostHeight / MinorVerticalGridResolution))));
            host.Background = new DrawingBrush(hostDrawingGroup);
        }

        private void ui_Inputs_Changed(object sender, SelectionChangedEventArgs e)
        {
            if (!this.IsLoaded) return;
            lock (locker)
            {
                baudRate = (int)cbxBaudRate.SelectedValue;
                timeScale = (double)cbxTimeScale.SelectedValue;
                dataReadRatio = (int)cbxReadRatio.SelectedValue;
                minorVerticalGridResolution = MajorVerticalGridResolution * MinorVerticalTicksPerMajor + 2;
                voltScale = (double)cbxVoltageScale.SelectedValue * minorVerticalGridResolution / (MajorVerticalGridResolution * MinorVerticalTicksPerMajor);
                if ((string)cbxVoltageBase.SelectedValue == "Bottom")
                {
                    voltMin = -voltScale / minorVerticalGridResolution;
                }
                else
                {
                    voltMin = 2.5 - voltScale / 2.0;
                }
            }

        }

        private void cbxPorts_DropDownOpened(object sender, EventArgs e)
        {
            string[] portNames = SerialPort.GetPortNames();
            cbxPorts.ItemsSource = portNames;
            if (portNames != null && portNames.Length > 0 && !string.IsNullOrEmpty(portNames[0])) cbxPorts.SelectedIndex = 0;
        }

        private void btnStart_Click(object sender, RoutedEventArgs e)
        {
            if (cbxPorts.SelectedItem != null && cbxPorts.SelectedItem is string && !backgroundWorker.IsBusy)
            {
                returnedData = null;
                backgroundWorker.RunWorkerAsync(cbxPorts.SelectedItem);
            }
        }

        private void btnStop_Click(object sender, RoutedEventArgs e)
        {
            if (backgroundWorker.IsBusy) backgroundWorker.CancelAsync();
        }

        

        /* 
        Third Version of DoWork
        This version works with Arduino Sketch: Oscilloscope2
        Assumptions regarding incoming data are the same as for the second version, DoWork2.
        Assumes that each arriving data packet consists of:
          The character 'b' (begin)
          byte containing the least significant 8 bits of time in microseconds
          byte containing:
            The most significant 6 bits of time in microseconds (as the least significant 6 bits in this byte)
            The least significant 2 bits of the A-to-D conversion result (as the most significant 2 bits in this byte)
          byte containing the most significant 8 bits of the A-to-D conversion

        Because the time in microseconds is sent as only 14 bits, it will "turn over" every 2^14 (max14Bit) microsends; this will
        manifest as a new time reading that is less than the old one.

        Triggering is re-worked. 

        Trigger Target Count no longer used. 
       
        When triggering is ON, trigger will happen when all of the following criteria are met: a)current volts > trigVolts;
        b) most recent prior volts < trigVolts; c) progressive voltage increases have occurred for the last three voltage 
        measurements (including the current one); d)The data has crossed the right end of the screen (so the screen always
        gets completely filled with data).

        If triggering is ON, but no threshhold crossing occurs, the current screen of data will be replaced with
        the next whole screen of data acquired.
        */
        private void backgroundWorker_DoWork3(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker bw = sender as BackgroundWorker;
            byte[] buf = new byte[3];
            DateTime priorReportingTime = DateTime.Now;
            int readRatio;
            long readCount = 0;
            double tScale;
            double trigVolts;
            List<OscilloscopeData> data1 = new List<OscilloscopeData>();
            List<OscilloscopeData> data2 = new List<OscilloscopeData>();
            bool trigOn;
            int bdRate;

            //Lock while reading in variables that belong to MainWindow
            lock (locker)
            {
                bdRate = baudRate;
                trigOn = triggerOn;
                readRatio = dataReadRatio;
                tScale = timeScale;
                trigVolts = triggerVolts;
            }
            ulong tScaleUsec = (ulong)(1000.0 * tScale);
            ulong tBase = 0;
            ulong oldDataTime = 0;

            using (SerialPort sp = new SerialPort((string)e.Argument, bdRate, Parity, DataBits, StopBits))
            {
                sp.Open();

                while (!bw.CancellationPending)
                {
                    //If data available, get next time,voltage reading
                    if (sp.BytesToRead < 4 || sp.ReadChar() != 'b' || sp.Read(buf, 0, 3) != 3) continue;
                    readCount++;

                    //Skip this reading if read interval not yet reached
                    if (readCount % readRatio != 0) continue;

                    //Extract the time in microseconds and the volts. tBase is added to the time to account for the accumulated
                    //"turnovers" of the 14 bit time data.
                    ulong dataTime = buf[0] + 256UL * (byte)(buf[1] & 0x3F) + tBase;
                    int dataValue = (buf[1] >> 6) + 4 * buf[2];
                    double volts = 5.0 * (double)dataValue / 1024.0;

                    //Determine whether a turnover of the 14 bit time has occurred. If so, increment tBase and the new dataTime by max22Bit
                    if (dataTime < oldDataTime)
                    {
                        tBase += max14Bit;
                        dataTime += max14Bit;
                    }

                    oldDataTime = dataTime;

                    //If triggering is turned on, datatime has exceeded time scale, and threshhold crossed, then handle trigger event
                    int last = data1.Count - 1;
                    if (trigOn && last > 1 && (dataTime - data1[0].Usec) > tScaleUsec && volts > trigVolts && data1[last].Volts < trigVolts)
                    {
                        //Triggering has occurred. New data1 will contain the data being collected starting with the current data point.
                        //data2 will contain the old data for backfilling. Its time values need to be increment by cycleTime so they appear
                        //after the data1 points.
                        data2 = data1;
                        data1 = new List<OscilloscopeData>() { new OscilloscopeData(dataTime, volts) };
                        ulong cycleTime = dataTime - data2[0].Usec;
                        for (int i = 0; i < data2.Count; i++) data2[i].Usec += cycleTime;
                    }
                    else
                    {
                        //Triggering has not occurred. Add the new data point to data1. If total time in data1 exceeds the timeScale AND triggering
                        // is turned off, OR if data1 contains an entire 2nd screen of data, then data2 will not be needed: clear it and trim beginning of data1
                        data1.Add(new OscilloscopeData(dataTime, volts));
                        ulong data1TotalUsec = dataTime - data1[0].Usec;
                        if ((data1TotalUsec > tScaleUsec && !trigOn) || data1TotalUsec > (2 * tScaleUsec))
                        {
                            data2.Clear();
                            while ((dataTime - data1[0].Usec) > tScaleUsec) data1.RemoveAt(0);
                        }
                    }

                    //Trim beginning of data2 (so it includes only data points that appear after all data in data1).
                    while (data2.Count > 0 && data2[0].Usec <= dataTime) data2.RemoveAt(0);

                    //If reporting interval has passed, report progress
                    DateTime currTime = DateTime.Now;
                    if ((currTime - priorReportingTime) > ProgressReportingInterval)
                    {
                        priorReportingTime = currTime;
                        double vMin;
                        double vScale;

                        //Lock while accessing variables that belong to MainWindow. Some of these are needed for reporting progress,
                        //while others are just being updated for use in future data acquisition
                        lock (locker)
                        {
                            //trigTargetCount = triggerTargetCount;
                            trigOn = triggerOn;
                            readRatio = dataReadRatio;
                            tScale = timeScale;
                            trigVolts = triggerVolts;
                            vMin = voltMin;
                            vScale = voltScale;
                        }

                        //Generate a list of x,y points for data1; also a list of x,y points for data2, if it contains at least two data points
                        //These list(s) are sent to ProgressChanged as a PointListPair for graphing. Only generate those points which are time scale.
                        tScaleUsec = (ulong)(1000.0 * tScale);
                        double tMin = (double)(data1[0].Usec) / 1000.0;
                        double tMax = tMin + tScale;
                        List<Point> points1 = new List<Point>();
                        for (int i = 0; i < data1.Count; i++)
                            if ((data1[i].Usec - data1[0].Usec) < tScaleUsec) points1.Add(getDisplayPoint(data1[i].Usec, data1[i].Volts, tMin, tScale, vMin, vScale));
                        PointListPair plp = new PointListPair(points1, null);

                        if (data2.Count > 1)
                        {
                            List<Point> points2 = new List<Point>();
                            for (int i = 1; i < data2.Count; i++)
                                if ((data2[i].Usec - data1[0].Usec) < tScaleUsec) points2.Add(getDisplayPoint(data2[i].Usec, data2[i].Volts, tMin, tScale, vMin, vScale));
                            plp.points2 = points2;
                        }
                        bw.ReportProgress(0, plp);
                    }
                }
            }
            data1.AddRange(data2);
            e.Result = data1;
        }


        private void backgroundWorker_DoWork4(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker bw = sender as BackgroundWorker;
            int voltageByte = 0;
            DateTime priorReportingTime = DateTime.Now;
            ulong readRatio;
            double uSecPerPixel;
            ulong readCount = 0;
            double tScale;
            double trigVolts;
            List<OscilloscopeData> data1 = new List<OscilloscopeData>();
            List<OscilloscopeData> data2 = new List<OscilloscopeData>();
            bool trigOn;
            int bdRate;

            //Lock while reading in variables that belong to MainWindow
            lock (locker)
            {
                bdRate = baudRate;
                trigOn = triggerOn;
                //readRatio = dataReadRatio;
                tScale = timeScale;
                trigVolts = triggerVolts;
            }
            ulong tScaleUsec = (ulong)(1000.0 * tScale);
            uSecPerPixel = tScaleUsec / HostWidth;
            readRatio = (ulong)(uSecPerPixel / acqTime);
            if (readRatio == 0) readRatio = 1;
            ulong dataTime = 0;

            using (SerialPort sp = new SerialPort((string)e.Argument, bdRate, Parity, DataBits, StopBits))
            {
                sp.Open();
                int[] buf = new int[] { 0, 0, 0 };

                while (!bw.CancellationPending)
                {
                    //If data available, get voltage reading
                    if (sp.BytesToRead == 0) continue;

                    buf[0] = buf[1];
                    buf[1] = buf[2];
                    buf[2] = sp.ReadByte();

                    dataTime += acqTime;

                    readCount++;

                    if (readCount % readRatio != 0) continue;
                    double volts = 5.0 * buf[1] / 256.0;
                    //double volts = 5.0 * (double)(buf[0] + 2 * buf[1] + buf[2]) / (256.0 * 4.0);
                    readCount++;

                    //If triggering is turned on, datatime has exceeded time scale, and threshhold crossed, then handle trigger event
                    int last = data1.Count - 1;
                    if (trigOn && last > 1 && (dataTime - data1[0].Usec) > tScaleUsec && volts > trigVolts && data1[last].Volts < trigVolts)
                    {
                        //Triggering has occurred. New data1 will contain the data being collected starting with the current data point.
                        //data2 will contain the old data for backfilling. Its time values need to be increment by cycleTime so they appear
                        //after the data1 points.
                        data2 = data1;
                        data1 = new List<OscilloscopeData>() { new OscilloscopeData(dataTime, volts) };
                        ulong cycleTime = dataTime - data2[0].Usec;
                        for (int i = 0; i < data2.Count; i++) data2[i].Usec += cycleTime;
                    }
                    else
                    {
                        //Triggering has not occurred. Add the new data point to data1. If total time in data1 exceeds the timeScale AND triggering
                        // is turned off, OR if data1 contains an entire 2nd screen of data, then data2 will not be needed: clear it and trim beginning of data1
                        data1.Add(new OscilloscopeData(dataTime, volts));
                        ulong data1TotalUsec = dataTime - data1[0].Usec;
                        if ((data1TotalUsec > tScaleUsec && !trigOn) || data1TotalUsec > (2 * tScaleUsec))
                        {
                            data2.Clear();
                            while ((dataTime - data1[0].Usec) > tScaleUsec) data1.RemoveAt(0);
                        }
                    }

                    //Trim beginning of data2 (so it includes only data points that appear after all data in data1).
                    while (data2.Count > 0 && data2[0].Usec <= dataTime) data2.RemoveAt(0);

                    //If reporting interval has passed, report progress
                    DateTime currTime = DateTime.Now;
                    if ((currTime - priorReportingTime) > ProgressReportingInterval)
                    {
                        priorReportingTime = currTime;
                        double vMin;
                        double vScale;

                        //Lock while accessing variables that belong to MainWindow. Some of these are needed for reporting progress,
                        //while others are just being updated for use in future data acquisition
                        lock (locker)
                        {
                            trigOn = triggerOn;
                            //readRatio = dataReadRatio;
                            tScale = timeScale;
                            trigVolts = triggerVolts;
                            vMin = voltMin;
                            vScale = voltScale;
                        }
                        tScaleUsec = (ulong)(1000.0 * tScale);
                        uSecPerPixel = tScaleUsec / HostWidth;
                        readRatio = (ulong)(uSecPerPixel / acqTime);
                        if (readRatio == 0) readRatio = 1;


                        //Generate a list of x,y points for data1; also a list of x,y points for data2, if it contains at least two data points
                        //These list(s) are sent to ProgressChanged as a PointListPair for graphing. Only generate those points which are time scale.
                        double tMin = (double)(data1[0].Usec) / 1000.0;
                        double tMax = tMin + tScale;
                        List<Point> points1 = new List<Point>();
                        for (int i = 0; i < data1.Count; i++)
                            if ((data1[i].Usec - data1[0].Usec) < tScaleUsec) points1.Add(getDisplayPoint(data1[i].Usec, data1[i].Volts, tMin, tScale, vMin, vScale));
                        PointListPair plp = new PointListPair(points1, null);

                        if (data2.Count > 1)
                        {
                            List<Point> points2 = new List<Point>();
                            for (int i = 0; i < data2.Count; i++)
                                if ((data2[i].Usec - data1[0].Usec) < tScaleUsec) points2.Add(getDisplayPoint(data2[i].Usec, data2[i].Volts, tMin, tScale, vMin, vScale));
                            plp.points2 = points2;
                        }
                        bw.ReportProgress(0, plp);
                    }
                }
            }
            data1.AddRange(data2);
            e.Result = data1;
        }



        private void backgroundWorker_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            PointListPair plp = (PointListPair)e.UserState;
            List<Point> points1 = plp.points1;
            List<Point> points2 = plp.points2;
            GeometryGroup geomGroup = new GeometryGroup();
            Point startPoint1 = points1[0];
            points1.RemoveAt(0);
            PolyLineSegment polyLnSeg1 = new PolyLineSegment(points1, true);
            PathFigure pathFig1 = new PathFigure(startPoint1, new PathSegment[] { polyLnSeg1 }, false);
            PathGeometry pathGeom1 = new PathGeometry(new PathFigure[] { pathFig1 });
            geomGroup.Children.Add(pathGeom1);
            if (points2 != null && points2.Count > 1)
            {
                Point startPoint2 = points2[0];
                points2.RemoveAt(0);
                PolyLineSegment polyLnSeg2 = new PolyLineSegment(points2, true);
                PathFigure pathFig2 = new PathFigure(startPoint2, new PathSegment[] { polyLnSeg2 }, false);
                PathGeometry pathGeom2 = new PathGeometry(new PathFigure[] { pathFig2 });
                geomGroup.Children.Add(pathGeom2);
            }
            GeometryDrawing geomDrawing = new GeometryDrawing(null, OscilloscopePen, geomGroup);
            host.DrawDrawing(geomDrawing);
        }

        private Point getDisplayPoint(ulong uSec, double volts, double tMin, double tScale, double vMin, double vScale)
        {
            double x = HostWidth * ((double)uSec / 1000.0 - tMin) / tScale;
            double y = HostHeight * (vMin + vScale - volts) / vScale;
            return new Point(x, y);
        }

        private void backgroundWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            if (e.Error != null)
            {
                MessageBox.Show(e.Error.Message, "Exception in backgroundWorker_DoWork");
                returnedData = null;
            }
            else if (e.Result == null || !(e.Result is List<OscilloscopeData>))
            {
                returnedData = null;
            }
            else returnedData = e.Result as List<OscilloscopeData>;
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            if (backgroundWorker.IsBusy) backgroundWorker.CancelAsync();
        }


        private void sldTrigVolts_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            lock (locker)
            {
                triggerVolts = sldTrigVolts.Value;
            }
        }
        

        private void btnSave_Click(object sender, RoutedEventArgs e)
        {
            if (backgroundWorker.IsBusy) return;
            if (returnedData == null || returnedData.Count == 0)
            {
                MessageBox.Show("No data available to save.");
                return;
            }

            SaveFileDialog dlg = new SaveFileDialog();
            dlg.OverwritePrompt = true;
            dlg.Filter = "Comma Separated Values Files (*.csv)|*.csv";

            if (dlg.ShowDialog() != true) return;

            try
            {
                using (StreamWriter writer = new StreamWriter(dlg.FileName))
                {
                    foreach (OscilloscopeData od in returnedData) writer.WriteLine("{0}, {1:G5}", od.Usec, od.Volts);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error writing file: " + ex.ToString());
            }
        }

        private void chkbxTriggerOn_Click(object sender, RoutedEventArgs e)
        {
            lock (locker)
            {
                triggerOn = (bool)chkbxTriggerOn.IsChecked;
            }
        }

        private List<OscilloscopeData> SmoothedAndSampled(List<OscilloscopeData> data, int w, int sampleInterval)
        {
            if (sampleInterval < 1) sampleInterval = 1;
            double[] q;
            double N;

            switch (w)
            {
                case 1:
                    q = new double[] { 1.0 };
                    N = 1.0;
                    break;
                case 2:
                    q = new double[] { 1.0, 2.0, 1.0 };
                    N = 4.0;
                    break;
                case 5:
                    q = new double[] { 1.0, 4.0, 6.0, 4.0, 1.0 };
                    N = 16.0;
                    break;
                case 10:
                    q = new double[] { 1.0, 6.0, 15.0, 20.0, 15.0, 6.0, 1.0 };
                    N = 64.0;
                    break;
                default:
                    q = new double[] { 1.0 };
                    N = 1.0;
                    break;
            }

            List<OscilloscopeData> returnData = new List<OscilloscopeData>();

            for (int i = 0; i < data.Count; i += sampleInterval)
            {
                double Ni = N;
                OscilloscopeData tempData = new OscilloscopeData(data[i].Usec, 0.0);
                for (int j = 0; j < q.Length; j++)
                {
                    int k = i - q.Length / 2 + j;
                    if (k < 0 || k >= data.Count) Ni -= q[j];
                    else tempData.Volts += q[j] * data[k].Volts;
                }
                tempData.Volts /= Ni;
                returnData.Add(tempData);
            }

            return returnData;
        }


    }



    class OscilloscopeData
    {
        public ulong Usec;
        public double Volts;

        public OscilloscopeData(ulong us, double v) { Usec = us; Volts = v; }

    }

    class PointListPair
    {
        public List<Point> points1;
        public List<Point> points2;
        public PointListPair( List<Point> p1, List<Point> p2 ) { points1 = p1;  points2 = p2; }
    }

    enum FrequencyMode { Normal, HiFreq}
}
