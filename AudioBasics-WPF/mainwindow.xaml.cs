//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace AudioBasics_WPF
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Windows;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow.
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Number of samples captured from Kinect audio stream each millisecond.
        /// </summary>
        private const int SamplesPerMillisecond = 16;

        /// <summary>
        /// Number of bytes in each Kinect audio stream sample (32-bit IEEE float).
        /// </summary>
        private const int BytesPerSample = sizeof(float);

        /// <summary>
        /// Number of audio samples represented by each column of pixels in wave bitmap.
        /// </summary>
        private const int SamplesPerColumn = 40;

        /// <summary>
        /// Will be allocated a buffer to hold a single sub frame of audio data read from audio stream.
        /// </summary>
        private readonly byte[] audioBuffer = null;

        private readonly float[] floatArray = null;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for audio frames
        /// </summary>
        private AudioBeamFrameReader reader = null;

        /// <summary>
        /// Sum of squares of audio samples being accumulated to compute the next energy value.
        /// </summary>
        private float accumulatedSquareSum;

        /// <summary>
        /// Number of audio samples accumulated so far to compute the next energy value.
        /// </summary>
        private int accumulatedSampleCount;

        private Pitch.PitchTracker pitchTracker;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // Initialize the components (controls) of the window
            this.InitializeComponent();

            // Only one Kinect Sensor is supported
            this.kinectSensor = KinectSensor.GetDefault();

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            if (this.kinectSensor != null)
            {
                // Open the sensor
                this.kinectSensor.Open();

                // Get its audio source
                AudioSource audioSource = this.kinectSensor.AudioSource;

                // Allocate 1024 bytes to hold a single audio sub frame. Duration sub frame 
                // is 16 msec, the sample rate is 16khz, which means 256 samples per sub frame. 
                // With 4 bytes per sample, that gives us 1024 bytes.
                this.audioBuffer = new byte[audioSource.SubFrameLengthInBytes];

                // Open the reader for the audio frames
                this.reader = audioSource.OpenReader();
            }
            else
            {
                // On failure, set the status text
                this.statusBarText.Text = Properties.Resources.NoSensorStatusText;
                return;
            }
            this.pitchTracker = new Pitch.PitchTracker();
            this.pitchTracker.SampleRate = 16000.0;
            this.floatArray = new float[this.audioBuffer.Length / 4];
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.reader != null)
            {
                // Subscribe to new audio frame arrived events
                this.reader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            //CompositionTarget.Rendering -= this.UpdateEnergy;

            if (this.reader != null)
            {
                // AudioBeamFrameReader is IDisposable
                this.reader.Dispose();
                this.reader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the audio frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
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

                        Console.WriteLine("     " + this.pitchTracker.CurrentPitchRecord.Pitch);
                        Console.WriteLine("     Hoi");

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
        
        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.statusBarText.Text = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}