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
//using System.Windows.Media;
//using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
//using System.Windows.Data;
//using System.Windows.Documents;
using System.Windows.Input;
//using System.Windows.Navigation;
//using System.Windows.Shapes;
using Microsoft.VisualBasic;
using Microsoft.VisualBasic.FileIO;

namespace HRI_project
{
    class HRIModel
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

        private Body[] skeletons = null;

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
        /// Current status text to display
        /// </summary>
        private string StatusText = null;

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
        
        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? "Kinect available" : "Kinect not available";
                                                           // : Properties.Resources.SensorNotAvailableStatusText;
        }
       
        public HRIModel ()
        {
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

            this.bones = createBonesList();
            
            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();
            IReadOnlyList<AudioBeam> audioBeamList = this.kinectSensor.AudioSource.AudioBeams;
            System.IO.Stream audioStream = audioBeamList[0].OpenInputStream();

            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Sensor_SkeletonFrameReady;
            }
            
            while (true)
            {
                if (skeletons != null)
                {
                    Console.WriteLine(AngleXY()[0]);
                    Console.WriteLine(AngleXY()[1]);
                    Console.WriteLine(Depth());
                }
            }

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
        }

        private List<Tuple<JointType, JointType>> createBonesList ()
        {
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

            return bones;
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
                    this.skeletons = new Body[frame.BodyCount];

                    frame.GetAndRefreshBodyData(this.skeletons);

                    var skeleton = skeletons.Where(s => s.IsTracked).FirstOrDefault();

                    if (skeleton != null)
                    {
                        double height = Height(skeleton)[0];
                    }
                }
            }
        }

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
                    foreach (Body body in this.bodies)
                    {
                        if (body.IsTracked)
                        {
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
                        }
                    }
                }
            }
    
        private void OutputMeasures(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, double[] bodym, Body body)
        {
           
            double bodyLength = (bodym[0]);
            double legLength = (bodym[1]); 
            double armLength = (bodym[2]);
            double shoulderWidth = (bodym[3]);
            double torso = (bodym[4]);
            //Point textLocbodyid = new Point(jointPoints[JointType.Head].X, jointPoints[JointType.Head].Y - 10);
         
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

        private float[] AngleXY() 
        {
            CameraSpacePoint xyHeadLocation;
            float[] angles = new float[2];
            float xAngle = 2000; 
            float yAngle = 2000;

            var skeleton = skeletons.Where(s => s.IsTracked).FirstOrDefault();
            if (skeleton != null)
            {
                var skeletonHead = skeleton.Joints[JointType.Head];
                xyHeadLocation = skeletonHead.Position;

                xAngle = xyHeadLocation.X;
                yAngle = xyHeadLocation.Y;
            }

            angles[0] = xAngle;
            angles[1] = yAngle;

            return angles;
        }

        private float Depth ()
        {
            CameraSpacePoint zSpineMidLocation;
            float depth = 2000;

            var skeleton = skeletons.Where(s => s.IsTracked).FirstOrDefault();
            if (skeleton != null)
            {
                var skeletonSpineMid = skeleton.Joints[JointType.SpineMid];
                zSpineMidLocation = skeletonSpineMid.Position;

                depth = zSpineMidLocation.Z;
            }

            return depth;
        }
    }
}
