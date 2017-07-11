using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.IO;

namespace bodyTrackingApp
{
    class Program
    {
        public static KinectSensor _sensor = null;
        public static MultiSourceFrameReader _multiSourceFrameReader = null;
        public static Worker _worker = new Worker();
        static void Main(string[] args)
        {
            _sensor = KinectSensor.GetDefault();
            
            if (_sensor != null)
            {
                _sensor.Open();
                Console.WriteLine("sensorOpened");
                if (_sensor.IsOpen)
                {
                    _worker.InitilizeMapper(_sensor);
                    _multiSourceFrameReader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);
                    _multiSourceFrameReader.MultiSourceFrameArrived += multiSourceReader_FrameArrived;
                }
            }
            string input = Console.ReadLine();
            _sensor.Close();
        }
        static void multiSourceReader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame _multiSourceFrame = e.FrameReference.AcquireFrame();
            if (_multiSourceFrame == null)
            {
                return;
            }
            if (_worker.counterFile == 0 && _worker.counterFrame == 0)
            {
                _worker.CreateFolderJoints();
                _worker.CreateFoldersImage();
                _worker.InilizeFileStream();
            }
            //Console.WriteLine(_worker.counterFrame);

            //long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            TimeSpan timeSpan = DateTime.Now.TimeOfDay;
            string timeNow = timeSpan.ToString();
            timeNow = timeNow.Replace('.', '_');
            timeNow = timeNow.Replace(':', '_');

            Task.Factory.StartNew(() =>
            {
                BodyFrame _bodyFrame = null;
                using (_bodyFrame = _multiSourceFrame.BodyFrameReference.AcquireFrame())
                {
                    if (_bodyFrame != null)
                    {
                        _bodyFrame.GetAndRefreshBodyData(_worker._bodies);
                        _worker.WriteBody(_worker._bodies, timeNow);
                    }
                }
            });

            Task.Factory.StartNew(() =>
            {
                BodyIndexFrame _bodyIndexFrame = null;
                using (_bodyIndexFrame = _multiSourceFrame.BodyIndexFrameReference.AcquireFrame())
                {
                    if (_bodyIndexFrame != null)
                    {
                        _worker.WriteBodyIndex(_bodyIndexFrame, timeNow);
                    }
                }
            });
            Task.Factory.StartNew(() =>
            {
                DepthFrame _depthFrame = null;
                using (_depthFrame = _multiSourceFrame.DepthFrameReference.AcquireFrame())
                {
                    if (_depthFrame != null)
                    {
                        _worker.WriteDepth(_depthFrame, timeNow);
                    }
                }
            });

            if (_worker.counterFrame++ >= 1000)
            {
                Console.WriteLine("999 frames have been saved..");
                _worker.counterFrame = 0;
                _worker.counterFile++;
                _worker.CreateFoldersImage();
                _worker.InilizeFileStream();
            }
        }
    }
}
