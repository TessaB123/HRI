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
using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;

namespace BodyHeight
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        public static double Height(this Body skeleton)
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


            // Find which leg is tracked more accurately.
            int legLeftTrackedJoints =
            NumberOfTrackedJoints(hipLeft, kneeLeft, ankleLeft, footLeft);
            int legRightTrackedJoints =
            NumberOfTrackedJoints(hipRight, kneeRight, ankleRight, footRight);


            double legLength = legLeftTrackedJoints > legRightTrackedJoints ?
              Length(hipLeft, kneeLeft, ankleLeft,
              footLeft) : Length(hipRight, kneeRight, ankleRight, footRight);


            return Length(head, neck, spine, waist) + legLength + HEAD_DIVERGENCE;
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
                        double height = Height(skeleton);
                    }
                }
            }
        }
       
    }
}