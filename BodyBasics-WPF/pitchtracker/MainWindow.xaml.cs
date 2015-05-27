using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Pitch;
using System.Windows.Threading;

namespace PitchTrackerSample
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private PitchTracker m_pitchTracker;
        private DispatcherTimer m_timer;
        private float[] m_audioBuffer;
        private int m_timeInterval;
        private float m_sampleRate;
        private double m_curWaveAngle;

        private readonly float m_minPitch = 55.0f;
        private readonly float m_maxPitch = 1500.0f;
        
        public MainWindow()
        {
            m_sampleRate = 44100.0f;
            m_timeInterval = 100;  // 100ms

            InitializeComponent();

            this.GeneratorPitch = 200.0f;
            this.GeneratorAmplitude = 0.1f;

            m_pitchTracker = new PitchTracker();
            m_pitchTracker.SampleRate = m_sampleRate;
            m_pitchTracker.PitchDetected += OnPitchDetected;

            m_audioBuffer = new float[(int)Math.Round(m_sampleRate * m_timeInterval / 1000.0)];

            UpdateDisplay();

            m_timer = new DispatcherTimer();
            m_timer.Interval = TimeSpan.FromMilliseconds(m_timeInterval);
            m_timer.Tick += OnTimerTick;
            m_timer.Start();
        }

        /// <summary>
        /// Get or set the current generator pitch
        /// </summary>
        private float GeneratorPitch
        {
            get
            {
                var sliderRatio = m_sliderPitch.Value / m_sliderPitch.Maximum;
                var maxVal = Math.Log10(m_maxPitch / m_minPitch);
                var pitch = (float)Math.Pow(10.0, sliderRatio * maxVal) * m_minPitch;

                return pitch;
            }

            set
            {
                if (value <= m_minPitch)
                {
                    m_sliderPitch.Value = m_sliderPitch.Minimum;
                }
                else if (value >= m_maxPitch)
                {
                    m_sliderPitch.Value = m_sliderPitch.Maximum;
                }
                else
                {
                    var maxVal = Math.Log10(m_maxPitch / m_minPitch);
                    var curVal = Math.Log10(value / m_minPitch);
                    var slider = m_sliderPitch.Maximum * curVal / maxVal;

                    m_sliderPitch.Value = slider;
                }
            }
        }

        /// <summary>
        /// Get or set the current generator amplitude
        /// </summary>
        private float GeneratorAmplitude
        {
            get { return (float)Math.Pow(10.0, m_sliderAmplitude.Value / 20.0); }
            set { m_sliderAmplitude.Value = 20.0 * Math.Log10(value); }
        }

        /// <summary>
        /// The timer ticked. This simulates a buffer that becomes ready at regular intervals,
        /// similar to how it works in a typical realtime application
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void OnTimerTick(object sender, EventArgs e)
        {
            m_curWaveAngle = PitchDsp.CreateSineWave(m_audioBuffer, m_audioBuffer.Length,
                m_sampleRate, this.GeneratorPitch, this.GeneratorAmplitude, m_curWaveAngle);

            m_pitchTracker.ProcessBuffer(m_audioBuffer);

            UpdateDisplay();
        }

        /// <summary>
        /// Update the display
        /// </summary>
        private void UpdateDisplay()
        {
            // Show the generator pitch
            var curPitch = this.GeneratorPitch;

            if (curPitch >= 1000.0f)
            {
                m_lblGeneratorPitch.Content = (curPitch / 1000.0f).ToString("F4");
                m_lblGeneratorPitchUnits.Content = "kHz";
            }
            else
            {
                m_lblGeneratorPitch.Content = curPitch.ToString("F3");
                m_lblGeneratorPitchUnits.Content = "Hz";
            }

            // Show the generator amplitude
            m_lblGeneratorLevel.Content = m_sliderAmplitude.Value.ToString("F1");

            // Show the detector results
            var curPitchRecord = m_pitchTracker.CurrentPitchRecord;

            if (curPitchRecord.Pitch > 1.0f)
            {
                if (curPitchRecord.Pitch >= 1000.0f)
                {
                    m_lblDetectorPitch.Content = (curPitchRecord.Pitch / 1000.0f).ToString("F4");
                    m_lblDetectorPitchUnits.Content = "kHz";
                }
                else
                {
                    m_lblDetectorPitch.Content = curPitchRecord.Pitch.ToString("F3");
                    m_lblDetectorPitchUnits.Content = "Hz";
                }
                
                m_lblDetectorMidiNote.Content = PitchDsp.GetNoteName(curPitchRecord.MidiNote, true, true);
                m_lblDetectorMidiCents.Content = curPitchRecord.MidiCents;

                var diffPercent = 100.0 - (100.0f * this.GeneratorPitch / curPitchRecord.Pitch);

                if (diffPercent >= 0.0f)
                    m_lblDetectorPitchError.Content = "+" + diffPercent.ToString("F3");
                else
                    m_lblDetectorPitchError.Content = diffPercent.ToString("F3");
            }
            else
            {
                m_lblDetectorPitch.Content = "--";
                m_lblDetectorPitchUnits.Content = "Hz";
                m_lblDetectorPitchError.Content = "--";
                m_lblDetectorMidiNote.Content = "--";
                m_lblDetectorMidiCents.Content = "--";
            }
        }

        /// <summary>
        /// The pitch was detected
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="pitchRecord"></param>
        private void OnPitchDetected(PitchTracker sender, PitchTracker.PitchRecord pitchRecord)
        {
            // During the call to PitchTracker.ProcessBuffer, this event will be fired zero or more times,
            // depending how many pitch records will fit in the new and previously cached buffer.
            //
            // This means that there is no size restriction on the buffer that is passed into ProcessBuffer.
            // For instance, ProcessBuffer can be called with one large buffer that contains all of the
            // audio to be processed, or just a small buffer at a time which is more typical for realtime
            // applications. This PitchDetected event will only occur once enough data has been accumulated
            // to do another detect operation.
        }

        /// <summary>
        /// Get the slider position from the pitch
        /// </summary>
        /// <param name="pitch"></param>
        /// <returns></returns>
        public double PitchToSlider(float pitch)
        {
            if (pitch <= m_minPitch)
                return m_sliderPitch.Minimum;

            if (pitch >= m_maxPitch)
                return m_sliderPitch.Maximum;

            var maxVal = Math.Log10(m_maxPitch / m_minPitch);
            var curVal = Math.Log10(pitch / m_minPitch);
            var slider = m_sliderPitch.Maximum * curVal / maxVal;

            return slider;
        }

        /// <summary>
        /// Get the pitch from the slider position
        /// </summary>
        /// <param name="pitch"></param>
        /// <returns></returns>
        public float SliderToPitch(double slider)
        {
            var sliderRatio = slider / m_sliderPitch.Maximum;
            var maxVal = Math.Log10(m_maxPitch / m_minPitch);
            var pitch = (float)Math.Pow(10.0, sliderRatio * maxVal) * m_minPitch;
            
            return pitch;
        }
    }
}
