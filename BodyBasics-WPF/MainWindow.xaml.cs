﻿//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
//https://msdn.microsoft.com/en-us/library/system.io.ports.serialport%28v=vs.110%29.aspx

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Net;
    using System.Net.Sockets;
    using System.Threading;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Linq;
    using System.Text;
    using System.Threading.Tasks;
    using System.Windows.Controls;
    using System.Windows.Data;
    using System.Windows.Documents;
    using System.Windows.Input;
    using System.Windows.Navigation;
    using System.Windows.Shapes;
    using Microsoft.VisualBasic;
    using Microsoft.VisualBasic.FileIO;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    /// 
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private Stream kinect32BitStream;
        private AudioBeamFrameReader reader = null;
        AudioSource audioSource;
        private float accumulatedSquareSum;
        private int accumulatedSampleCount;
        private byte[] packetData;
        Pitch.PitchTracker pitchTracker;
        private const int BytesPerSample = sizeof(float);


        private const int SamplesPerColumn = 40;


        byte[] audioBuffer = null;
        float[] floatArray = null;
        string IP;
        int port;
        private IPEndPoint ep;
        Socket client;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            //initialyse socket
            this.IP = "127.0.0.1";
            this.port = 80;
            this.ep = new IPEndPoint(IPAddress.Parse(this.IP), port);
            this.client = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            //client.sentTo(pakcetData, ep)
            // client.timeout(1)

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();
            IReadOnlyList<AudioBeam> audioBeamList = this.kinectSensor.AudioSource.AudioBeams;
            System.IO.Stream audioStream = audioBeamList[0].OpenInputStream();


            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            // set pitchtracker
            audioSource = this.kinectSensor.AudioSource;
            audioBuffer = new byte[audioSource.SubFrameLengthInBytes];
            floatArray = new float[audioBuffer.Length / 4];
            this.reader = audioSource.OpenReader();
            pitchTracker = new Pitch.PitchTracker();
            pitchTracker.SampleRate = 16000.0;
            if (this.reader != null)
            {
                // Subscribe to new audio frame arrived events
                this.reader.FrameArrived += this.Reader_FrameArrived;
            }




            // Allocate 1024 bytes to hold a single audio sub frame. Duration sub frame 
            // is 16 msec, the sample rate is 16khz, which means 256 samples per sub frame. 
            // With 4 bytes per sample, that gives us 1024 bytes.
            //this.kinect32BitStream = input;
        }

        private void Reader_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
        {
            AudioBeamFrameReference frameReference = e.FrameReference;
            AudioBeamFrameList frameList = frameReference.AcquireBeamFrames();

            if (frameList != null)
            {
                // AudioBeamFrameList is IDisposable
                using (frameList)
                {
                    // Only one audio beam is supported. Get the sub frame list for this beam
                    IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                    // Loop over all sub frames, extract audio buffer and beam information
                    foreach (AudioBeamSubFrame subFrame in subFrameList)
                    {
                        // Process audio buffer
                        subFrame.CopyFrameDataToArray(this.audioBuffer);

                        Buffer.BlockCopy(this.audioBuffer, 0, this.floatArray, 0, this.audioBuffer.Length);
                        this.pitchTracker.ProcessBuffer(this.floatArray);
                        Console.WriteLine(subFrame.BeamAngle * 180.0f / (float)Math.PI);
                        IReadOnlyList<AudioBodyCorrelation> asdf = subFrame.AudioBodyCorrelations;
                        //  Console.WriteLine("     " + this.pitchTracker.CurrentPitchRecord.Pitch);
                        //  Console.WriteLine("     Hoi");

                        for (int i = 0; i < this.audioBuffer.Length; i += BytesPerSample)
                        {
                            // Extract the 32-bit IEEE float sample from the byte array
                            float audioSample = BitConverter.ToSingle(this.audioBuffer, i);

                            this.accumulatedSquareSum += audioSample * audioSample;
                            ++this.accumulatedSampleCount;

                            if (this.accumulatedSampleCount < SamplesPerColumn)
                            {
                                continue;
                            }

                            float meanSquare = this.accumulatedSquareSum / SamplesPerColumn;

                            if (meanSquare > 1.0f)
                            {
                                // A loud audio source right next to the sensor may result in mean square values
                                // greater than 1.0. Cap it at 1.0f for display purposes.
                                meanSquare = 1.0f;
                            }

                            this.accumulatedSquareSum = 0;
                            this.accumulatedSampleCount = 0;
                        }
                    }
                }
            }
        }

        public Thread StartTheThread(AudioSource audioSource)
        {

            var t = new Thread(() => RealStart(audioSource));
            t.Start();
            return t;
        }
        private void RealStart(AudioSource audioSource)
        {

            Pitch.PitchTracker pitchTracker;
            pitchTracker = new Pitch.PitchTracker();
            pitchTracker.SampleRate = 16000.0;

            byte[] audioBuffer = null;
            float[] floatArray = null;
            while (!_shouldStop)
            {
                audioBuffer = new byte[audioSource.SubFrameLengthInBytes];
                floatArray = new float[audioBuffer.Length / 4];
                pitchTracker.ProcessBuffer(floatArray);
                if (pitchTracker.CurrentPitchRecord.Pitch > 0)
                    Console.WriteLine("     " + pitchTracker.CurrentPitchRecord.Pitch);

            }


        }
        public void RequistStop()
        {
            _shouldStop = true;
        }
        private volatile bool _shouldStop;


        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {


            // csv writer
            //var csvw = new StreamWriter("CSV.csv");
            // csv reader
            // var csv = new StreamReader("CSV.csv");

            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    // tblHeights.Text = string.Empty;
                    foreach (Body body in this.bodies)
                    {

                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                            }
                            this.DrawOutputMeasures(joints, jointPoints, dc, drawPen, Height(body), body);
                            this.DrawBody(joints, jointPoints, dc, drawPen);
                            //joints[JointType.Neck].Position.
                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }

                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        private void DrawOutputMeasures(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen, double[] bodym, Body body)
        {
            string testString = "";
            double bodyLength = (bodym[0]);


            // ToString(Math.Round(Height(body), 2));
            testString = Math.Round(bodyLength, 2).ToString(); //string.Format("", body.TrackingId, );
            FormattedText formattedText = new FormattedText("lengte: " + testString, CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Comic Sans"), 12, drawingPen.Brush);
            Point textLoc = new Point(jointPoints[JointType.Head].X + 100, jointPoints[JointType.Head].Y);
            drawingContext.DrawText(formattedText, textLoc);

            double legLength = (bodym[1]);
            testString = Math.Round(legLength, 2).ToString(); //string.Format("", body.TrackingId, );
            FormattedText formattedTextLeg = new FormattedText("beenlengte: " + testString, CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Comic Sans"), 12, drawingPen.Brush);
            Point textLocLeg = new Point(jointPoints[JointType.KneeLeft].X + 100, jointPoints[JointType.KneeLeft].Y);
            drawingContext.DrawText(formattedTextLeg, textLocLeg);

            double armLength = (bodym[2]);
            testString = Math.Round(armLength, 2).ToString(); //string.Format("", body.TrackingId, );
            FormattedText formattedTextArm = new FormattedText("armlengte: " + testString, CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Comic Sans"), 12, drawingPen.Brush);
            Point textLocArm = new Point(jointPoints[JointType.ElbowLeft].X + 100, jointPoints[JointType.ElbowLeft].Y);
            drawingContext.DrawText(formattedTextArm, textLocArm);

            double shoulderWidth = (bodym[3]);
            testString = Math.Round(shoulderWidth, 2).ToString(); //string.Format("", body.TrackingId, );
            FormattedText formattedTextShoulder = new FormattedText("schouderbreedte: " + testString, CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Comic Sans"), 12, drawingPen.Brush);
            Point textLocShoulder = new Point(jointPoints[JointType.ShoulderLeft].X + 100, jointPoints[JointType.ShoulderLeft].Y);
            drawingContext.DrawText(formattedTextShoulder, textLocShoulder);

            double torso = (bodym[4]);
            testString = Math.Round(torso, 2).ToString(); //string.Format("", body.TrackingId, );
            FormattedText formattedTextTorso = new FormattedText("torso: " + testString, CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Comic Sans"), 12, drawingPen.Brush);
            Point textLocTorso = new Point(jointPoints[JointType.SpineMid].X + 100, jointPoints[JointType.SpineMid].Y);
            drawingContext.DrawText(formattedTextTorso, textLocTorso);



            FormattedText formattedTextbodyid = new FormattedText("ID: " + body.TrackingId, CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Comic Sans"), 12, drawingPen.Brush);
            Point textLocbodyid = new Point(jointPoints[JointType.Head].X, jointPoints[JointType.Head].Y - 10);
            drawingContext.DrawText(formattedTextbodyid, textLocbodyid);

            bool recognised = false;
            Double counter = 0;
            bool empty = true;
            List<List<string>> data = new List<List<string>>();
            List<string> values = new List<string>();
            string[][] output = new string[][] { new string[] { "" } };
            for (int i = 0; i < bodym.Length; i++)
            {
                if (bodym[i] == 0)
                {
                    empty = false;
                }
            }
            if (empty)
            {
                string path = @"unknownPeople.csv";
                string path2 = @"knownPeople.csv";
                string name = "";
                string delimiter = ";";
                if (File.Exists(path) && counter < 101)
                {
                    string text = File.ReadAllText(path);
                    text = text.Replace("\r\n", "\n");
                    int l = text.Length;
                    for (int i = l - 1; i >= 0; i--)
                    {

                        if (text[i] == ';')
                        {
                            String a = text.Substring(i + 1);
                            values.Add(a);
                            text = text.Remove(i + 1);

                            text = text.Remove(i);
                        }
                        else if (text[i] == '\n' && counter != 0)
                        {

                            String d = text.Substring(i + 1);
                            values.Add(d);
                            text = text.Remove(i + 1);
                            text = text.Remove(i);
                        }
                    }


                    data.Add(values);
                    values = new List<string>();
                    string sdfae = data[0][0];
                    counter = Convert.ToDouble(data[0][0].Replace("\n", string.Empty)) + 1;
                    Console.Write(counter);
                }
                else
                {
                    Console.Write("counter =0");
                    counter = 0;
                }
                if (counter == 0.0)
                {
                    output = new string[][] { new string[] { "" + body.TrackingId, (bodyLength * 1000).ToString(), (legLength * 1000).ToString(), (armLength * 1000).ToString(), (shoulderWidth * 1000).ToString(), (torso * 1000).ToString(), counter.ToString() } };
                    int length2 = output.GetLength(0);
                    StringBuilder sb = new StringBuilder();
                    for (int i = 0; i < length2; i++)
                    {
                        sb.AppendLine(string.Join(delimiter, output[i]));
                    }
                    File.WriteAllText(path, sb.ToString());
                }
                else
                {

                    output = new string[][] { new string[] { "" + body.TrackingId, ((Convert.ToDouble(data[0][5]) * (counter - 1) + bodyLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][4]) * (counter - 1) + legLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][3]) * (counter - 1) + armLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][2]) * (counter - 1) + shoulderWidth * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][1]) * (counter - 1) + torso * 1000) / counter).ToString(), counter.ToString() } };
                }
                if (counter > 100)
                {
                    name = recognise(output, path2);
                    if (name != "")
                    { recognised = true; }
                    if (!recognised)
                    {
                        //name = InputName();
                        name = body.TrackingId + "";
                        output = new string[][] { new string[] { "" + body.TrackingId, ((Convert.ToDouble(data[0][5]) * (counter - 1) + bodyLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][4]) * (counter - 1) + legLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][3]) * (counter - 1) + armLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][2]) * (counter - 1) + shoulderWidth * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][1]) * (counter - 1) + torso * 1000) / counter).ToString(), counter.ToString() } };
                        int length = output.GetLength(0);
                        StringBuilder sb2 = new StringBuilder();
                        for (int i = 0; i < length; i++)
                        {
                            sb2.AppendLine(string.Join(delimiter, output[i]));
                        }
                        if (File.Exists(path2))
                        { File.WriteAllText(path2, File.ReadAllText(path2) + sb2.ToString()); }
                        else
                        { File.WriteAllText(path2, sb2.ToString()); }


                    }
                    else
                    {

                        output = new string[][] { new string[] { "" + body.TrackingId, ((Convert.ToDouble(data[0][5]) * (counter - 1) + bodyLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][4]) * (counter - 1) + legLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][3]) * (counter - 1) + armLength * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][2]) * (counter - 1) + shoulderWidth * 1000) / counter).ToString(), ((Convert.ToDouble(data[0][1]) * (counter - 1) + torso * 1000) / counter).ToString(), counter.ToString() } };

                    }

                }
                if (!recognised)
                {
                    int length = output.GetLength(0);
                    StringBuilder sb2 = new StringBuilder();
                    for (int i = 0; i < length; i++)
                    {
                        sb2.AppendLine(string.Join(delimiter, output[i]));
                    }

                    if (File.Exists(path))
                    {
                        File.WriteAllText(path, File.ReadAllText(path) + sb2.ToString());

                    }
                    else { File.WriteAllText(path, File.ReadAllText(path) + sb2.ToString()); }

                    if (File.Exists(path))
                    {
                        File.WriteAllText(path, sb2.ToString());

                    }
                    else { File.WriteAllText(path, sb2.ToString()); }
                }
            }
        }


        private string recognise(string[][] output, string path)
        {

            double distance = 999999999999999999999999999999999999999999.0;
            string name = "";
            double smallestDist = 999999999999999999999.0;
            double sum = 0;
            if (File.Exists(path))
            {
                using (TextFieldParser parser = new TextFieldParser(path))
                {
                    parser.TextFieldType = FieldType.Delimited;
                    parser.SetDelimiters(";");
                    while (!parser.EndOfData)
                    {
                        //Processing row
                        string[] fields = parser.ReadFields();
                        for (int i = 1; i < fields.Length - 1; i++)
                        {
                            double a = Convert.ToDouble(fields[i]);
                            double b = Convert.ToDouble(output[0][i]);
                            double c = (a - b);
                            double asdflh = Math.Pow(c, 2);
                            sum += asdflh;
                        }

                        distance = Math.Sqrt(sum);
                        sum = 0;
                        if (smallestDist > distance)
                        {
                            smallestDist = distance;
                            name = fields[0];
                        }
                    }
                    parser.Close();
                }
            }
            if (smallestDist < 0.01)
            { return name; }
            else { return ""; }

        }


        private string InputName()
        {
            return "";
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        public static double[] Height(Body skeleton)
        {
            const double HEAD_DIVERGENCE = 0.1;

            var head = skeleton.Joints[JointType.Head];
            var neck = skeleton.Joints[JointType.SpineShoulder];
            var spine = skeleton.Joints[JointType.SpineMid];
            var waist = skeleton.Joints[JointType.SpineBase];
            var hipLeft = skeleton.Joints[JointType.HipLeft];
            var hipRight = skeleton.Joints[JointType.HipRight];
            var kneeLeft = skeleton.Joints[JointType.KneeLeft];
            var kneeRight = skeleton.Joints[JointType.KneeRight];
            var ankleLeft = skeleton.Joints[JointType.AnkleLeft];
            var ankleRight = skeleton.Joints[JointType.AnkleRight];
            var footLeft = skeleton.Joints[JointType.FootLeft];
            var footRight = skeleton.Joints[JointType.FootRight];
            var shoulderLeft = skeleton.Joints[JointType.ShoulderLeft];
            var shoulderRight = skeleton.Joints[JointType.ShoulderRight];
            var elbowLeft = skeleton.Joints[JointType.ElbowLeft];
            var elbowRight = skeleton.Joints[JointType.ElbowRight];
            var wristLeft = skeleton.Joints[JointType.WristLeft];
            var wristRight = skeleton.Joints[JointType.WristRight];
            var handLeft = skeleton.Joints[JointType.HandLeft];
            var handRight = skeleton.Joints[JointType.HandRight];
            var handTipLeft = skeleton.Joints[JointType.HandTipLeft];
            var handTipRight = skeleton.Joints[JointType.HandTipRight];



            // Find which leg is tracked more accurately.
            int legLeftTrackedJoints =
            NumberOfTrackedJoints(hipLeft, kneeLeft, ankleLeft, footLeft);
            int legRightTrackedJoints =
            NumberOfTrackedJoints(hipRight, kneeRight, ankleRight, footRight);
            int armLeftTrackedJoints = NumberOfTrackedJoints(shoulderLeft, elbowLeft, wristLeft, handLeft, handTipLeft);
            int armRightTrackedJoints = NumberOfTrackedJoints(shoulderRight, elbowRight, wristRight, handRight, handTipRight);

            double shoulderWidth = Length(shoulderLeft, neck, shoulderRight);
            double torso = Length(neck, spine, waist);

            double legLength = legLeftTrackedJoints > legRightTrackedJoints ?
              Length(hipLeft, kneeLeft, ankleLeft,
              footLeft) : Length(hipRight, kneeRight, ankleRight, footRight);

            double armLength = armLeftTrackedJoints > armRightTrackedJoints ?
                Length(shoulderLeft, elbowLeft, wristLeft, handLeft, handTipLeft)
                : Length(shoulderRight, elbowRight, wristRight, handRight, handTipRight);

            double[] buh = { Length(head, neck, spine, waist) + legLength + HEAD_DIVERGENCE, legLength, armLength, shoulderWidth, torso };

            return buh;
        }

        public static int NumberOfTrackedJoints(params Joint[] joints)
        {
            int trackedJoints = 0;

            foreach (var joint in joints)
            {
                if (joint.TrackingState == TrackingState.Tracked)
                {
                    trackedJoints++;
                }
            }


            return trackedJoints;
        }

        public static double Length(Joint p1, Joint p2)
        {
            return Math.Sqrt(
                Math.Pow(p1.Position.X - p2.Position.X, 2) +
                Math.Pow(p1.Position.Y - p2.Position.Y, 2) +
                Math.Pow(p1.Position.Z - p2.Position.Z, 2));
        }

        public static double Length(params Joint[] joints)
        {
            double length = 0;

            for (int index = 0; index < joints.Length - 1; index++)
            {
                length += Length(joints[index], joints[index + 1]);
            }


            return length;
        }

        void Sensor_SkeletonFrameReady(object sender, BodyFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    Body[] skeletons = new Body[frame.BodyCount];
                    frame.GetAndRefreshBodyData(skeletons);

                    var skeleton = skeletons.Where(s => s.IsTracked).FirstOrDefault();

                    if (skeleton != null)
                    {
                        double height = Height(skeleton)[0];
                    }
                }
            }
        }







    }

    /// <summary>
    /// Wrapper Stream Class to Support 32->16bit conversion and support Speech call to Seek
    /// </summary>
    //    internal class KinectAudioStream : Stream
    //    {
    //        /// <summary>
    //        /// Holds the kinect audio stream, in 32-bit IEEE float format
    //        /// </summary>
    //        private Stream kinect32BitStream;

    //        /// <summary>
    //        /// Initializes a new instance of the <see cref="KinectAudioStream" /> class.
    //        /// </summary>
    //        /// <param name="input">Kinect audio stream</param>
    //        public KinectAudioStream(Stream input)
    //        {
    //            this.kinect32BitStream = input;
    //        }

    //        /// <summary>
    //        /// Gets or sets a value indicating whether speech recognition is active
    //        /// </summary>
    //        public bool SpeechActive { get; set; }

    //        /// <summary>
    //        /// CanRead property
    //        /// </summary>
    //        public override bool CanRead
    //        {
    //            get { return true; }
    //        }

    //        /// <summary>
    //        /// CanWrite property
    //        /// </summary>
    //        public override bool CanWrite
    //        {
    //            get { return false; }
    //        }

    //        /// <summary>
    //        /// CanSeek property
    //        /// </summary>
    //        public override bool CanSeek
    //        {
    //            // Speech does not call - but set value correctly
    //            get { return false; }
    //        }

    //        /// <summary>
    //        /// Position Property
    //        /// </summary>
    //        public override long Position
    //        {
    //            // Speech gets the position
    //            get { return 0; }
    //            set { throw new NotImplementedException(); }
    //        }

    //        /// <summary>
    //        /// Gets the length of the stream. Not implemented.
    //        /// </summary>
    //        public override long Length
    //        {
    //            get { throw new NotImplementedException(); }
    //        }

    //        /// <summary>
    //        /// Flush the stream. Not implemented.
    //        /// </summary>
    //        public override void Flush()
    //        {
    //            throw new NotImplementedException();
    //        }

    //        /// <summary>
    //        /// Stream Seek. Not implemented and always returns 0.
    //        /// </summary>
    //        /// <param name="offset">A byte offset relative to the origin parameter</param>
    //        /// <param name="origin">A value of type SeekOrigin indicating the reference point used to obtain the new position</param>
    //        /// <returns>Always returns 0</returns>
    //        public override long Seek(long offset, SeekOrigin origin)
    //        {
    //            // Even though CanSeek == false, Speech still calls seek. Return 0 to make Speech happy instead of NotImplementedException()
    //            return 0;
    //        }

    //        /// <summary>
    //        /// Set the length of the stream. Not implemented.
    //        /// </summary>
    //        /// <param name="value">Length of the stream</param>
    //        public override void SetLength(long value)
    //        {
    //            throw new NotImplementedException();
    //        }

    //        /// <summary>
    //        /// Write into the stream. Not implemented.
    //        /// </summary>
    //        /// <param name="buffer">Buffer to write</param>
    //        /// <param name="offset">Offset into the buffer</param>
    //        /// <param name="count">Number of bytes to write</param>
    //        public override void Write(byte[] buffer, int offset, int count)
    //        {
    //            throw new NotImplementedException();
    //        }

    //        /// <summary>
    //        /// Read from the stream and convert from 32 bit IEEE float to 16 bit signed integer
    //        /// </summary>
    //        /// <param name="buffer">Input buffer</param>
    //        /// <param name="offset">Offset into buffer</param>
    //        /// <param name="count">Number of bytes to read</param>
    //        /// <returns>bytes read</returns>
    //        public override int Read(byte[] buffer, int offset, int count)
    //        {
    //            // Kinect gives 32-bit float samples. Speech asks for 16-bit integer samples.
    //            const int SampleSizeRatio = sizeof(float) / sizeof(short); // = 2. 

    //            // Speech reads at high frequency - allow some wait period between reads (in msec)
    //            const int SleepDuration = 50;

    //            // Allocate buffer for receiving 32-bit float from Kinect
    //            int readcount = count * SampleSizeRatio;
    //            byte[] kinectBuffer = new byte[readcount];

    //            int bytesremaining = readcount;

    //            // Speech expects all requested bytes to be returned
    //            while (bytesremaining > 0)
    //            {
    //                // If we are no longer processing speech commands, exit
    //                if (!this.SpeechActive)
    //                {
    //                    return 0;
    //                }

    //                int result = this.kinect32BitStream.Read(kinectBuffer, readcount - bytesremaining, bytesremaining);
    //                bytesremaining -= result;

    //                // Speech will read faster than realtime - wait for more data to arrive
    //                if (bytesremaining > 0)
    //                {
    //                    System.Threading.Thread.Sleep(SleepDuration);
    //                }
    //            }

    //            // Convert each float audio sample to short
    //            for (int i = 0; i < count / sizeof(short); i++)
    //            {
    //                // Extract a single 32-bit IEEE value from the byte array
    //                float sample = BitConverter.ToSingle(kinectBuffer, i * sizeof(float));

    //                // Make sure it is in the range [-1, +1]
    //                if (sample > 1.0f)
    //                {
    //                    sample = 1.0f;
    //                }
    //                else if (sample < -1.0f)
    //                {
    //                    sample = -1.0f;
    //                }

    //                // Scale float to the range (short.MinValue, short.MaxValue] and then 
    //                // convert to 16-bit signed with proper rounding
    //                short convertedSample = Convert.ToInt16(sample * short.MaxValue);

    //                // Place the resulting 16-bit sample in the output byte array
    //                byte[] local = BitConverter.GetBytes(convertedSample);
    //                System.Buffer.BlockCopy(local, 0, buffer, offset + (i * sizeof(short)), sizeof(short));
    //            }

    //            return count;
    //        }
    //    }


    //}
}
