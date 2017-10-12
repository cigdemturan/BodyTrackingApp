using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using System.Xml;
using System.IO;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.Windows;
using System.Drawing;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows;
using System.Runtime.InteropServices;

namespace bodyTrackingApp
{
    class Worker
    {
        public string _fileName { get; set; }
        private string _folderNameRec = "recording";
        public string _jointsFolder, _depthFolder, _bodyIndexFolder;
        public int counterFile = 0;
        public int counterFrame = 0;
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        private readonly int bytesPerPixelBI = (PixelFormats.Gray8.BitsPerPixel + 7) / 8;
        public bool flagNoBody = true;
        public FrameDescription depthFrameDescription { get; set; }
        public Body[] _bodies = new Body[6];
        private FileStream jointsFileStream = null;
        private FileStream jointsFileStream2D = null;
        private CoordinateMapper _coordinateMapper = null;

        public void getSubjectID()
        {
            Console.WriteLine("enter the subject ID");
            string input = Console.ReadLine();
            _folderNameRec = Path.Combine(@"C:\NOISE_PATTERN\SAVINGS\", input);
            if (!Directory.Exists(_folderNameRec))
            {
                Directory.CreateDirectory(_folderNameRec);
                Console.WriteLine("folder is created..");
            }
        }

        public void InitilizeMapper(KinectSensor sensor)
        {
            _coordinateMapper = sensor.CoordinateMapper;
        }

        public void CreateFolderJoints()
        {
            _jointsFolder = _folderNameRec + "/joints/";
            Directory.CreateDirectory(_jointsFolder);
        }
        public void CreateFoldersImage()
        {
            _depthFolder = _folderNameRec + "/depth_" + counterFile + "/";
            Directory.CreateDirectory(_depthFolder);
            _bodyIndexFolder = _folderNameRec + "/bodyIndex_" + counterFile + "/";
            Directory.CreateDirectory(_bodyIndexFolder);
        }
        public void InilizeFileStream()
        {
            //long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            TimeSpan timeSpan = DateTime.Now.TimeOfDay;
            string timeNow = timeSpan.ToString();
            timeNow = timeNow.Replace('.', '_');
            timeNow = timeNow.Replace(':', '_');
            string jointsFile = _jointsFolder + "joints3D_" + counterFile + "_" + timeNow + "_.txt";
            jointsFileStream = new FileStream(jointsFile, FileMode.OpenOrCreate, FileAccess.ReadWrite);
            string jointsFile2D = _jointsFolder + "joints2D_" + counterFile + "_" + timeNow + "_.txt";
            jointsFileStream2D = new FileStream(jointsFile2D, FileMode.OpenOrCreate, FileAccess.ReadWrite);
        }
        public void WriteBody(Body[] _bodies, string milliseconds)
        {
            var _jointsStreamWriter3D = new StreamWriter(jointsFileStream);
            _jointsStreamWriter3D.Write("\r\n{0}, ", milliseconds);
            var _jointsStreamWriter2D = new StreamWriter(jointsFileStream2D);
            _jointsStreamWriter2D.Write("\r\n{0}, ", milliseconds);
            int countB = 0;

            Body _bodyTracked = null;

            for (int b = 0; b < 6; b++)
            {
                Body body = _bodies[b];
                if (body != null & body.IsTracked)
                {
                    countB++;
                    _bodyTracked = body;
                    if (flagNoBody)
                    {
                        flagNoBody = false;
                        Console.WriteLine("start tracking the body...");
                    }
                }
            }

            if (countB == 1)
            {
                foreach (JointType _joint in _bodyTracked.Joints.Keys)
                {
                    CameraSpacePoint position3D = _bodyTracked.Joints[_joint].Position;
                    DepthSpacePoint position2D = _coordinateMapper.MapCameraPointToDepthSpace(position3D);
                    //_jointsStreamWriter.Write("{0}, ", body.Joints[_joint].JointType);
                    _jointsStreamWriter3D.Write("{0}, ", position3D.X);
                    _jointsStreamWriter3D.Write("{0}, ", position3D.Y);
                    _jointsStreamWriter3D.Write("{0}, ", position3D.Z);
                    //Console.WriteLine(body.Joints[_joint].JointType);
                    _jointsStreamWriter2D.Write("{0}, ", position2D.X);
                    _jointsStreamWriter2D.Write("{0}, ", position2D.Y);
                }
            }
            else
            {
                if (!flagNoBody)
                {
                    Console.WriteLine("lost the body...");
                    flagNoBody = true;
                }
                _jointsStreamWriter3D.Write("NaN, ");
                _jointsStreamWriter2D.Write("NaN, ");
            }

            _jointsStreamWriter3D.Flush();
            _jointsStreamWriter2D.Flush();

        }
        public void WriteDepth(DepthFrame _depthFrame, string milliseconds)
        {
            depthFrameDescription = _depthFrame.FrameDescription;
            uint depthSize = depthFrameDescription.LengthInPixels;
            byte[] pixelsDepth = null;
            ushort[] depthData = new ushort[depthFrameDescription.Width * depthFrameDescription.Height]; //Depth
            pixelsDepth = new byte[depthFrameDescription.Width * depthFrameDescription.Height * bytesPerPixel]; //Depth
            _depthFrame.CopyFrameDataToArray(depthData);


            ushort minDepth = 1000;
            ushort maxDepth = 3000;
            //ushort minDepth = _depthFrame.DepthMinReliableDistance;
            //ushort maxDepth = _depthFrame.DepthMaxReliableDistance;
            //Console.WriteLine(minDepth);
            //Console.WriteLine(maxDepth);

            int colorImageIndex = 0;
            for (int depthIndex = 0; depthIndex < depthData.Length; ++depthIndex)
            {
                ushort depth = depthData[depthIndex];
                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                pixelsDepth[colorImageIndex++] = intensity; // Blue
                pixelsDepth[colorImageIndex++] = intensity; // Green
                pixelsDepth[colorImageIndex++] = intensity; // Red

                ++colorImageIndex;
            }

            int stride = depthFrameDescription.Width * PixelFormats.Bgr32.BitsPerPixel / 8;
            
            BitmapSource _imageDepth = BitmapSource.Create(depthFrameDescription.Width, depthFrameDescription.Height, 96, 96, PixelFormats.Bgr32, null, pixelsDepth, stride);
            //Console.WriteLine("here");
            string filePath = _depthFolder + "/image" + milliseconds + ".png";

            using (FileStream depth_FileStream = new FileStream(filePath, FileMode.Create))
            {
                BitmapEncoder encoder = new PngBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(_imageDepth));
                encoder.Save(depth_FileStream);
                //Console.WriteLine("saved..");
                depth_FileStream.Close();
                depth_FileStream.Dispose();
            }
            _depthFrame.Dispose();   
        }

        public void WriteBodyIndex(BodyIndexFrame _bodyIndexFrame, string milliseconds)
        {
            FrameDescription bodyIndexFrameDescription = _bodyIndexFrame.FrameDescription;
            //Console.WriteLine("colorFrame is here");
            byte[] pixelsBodyIndex = new byte[bodyIndexFrameDescription.Width * bodyIndexFrameDescription.Height * bytesPerPixelBI]; //Body Index
            _bodyIndexFrame.CopyFrameDataToArray(pixelsBodyIndex);
            int stride = bodyIndexFrameDescription.Width * PixelFormats.Gray8.BitsPerPixel / 8;

            BitmapSource _imageBI = BitmapSource.Create(bodyIndexFrameDescription.Width, bodyIndexFrameDescription.Height, 96, 96, PixelFormats.Gray8, null, pixelsBodyIndex, stride);
            string filePath = _bodyIndexFolder + "/image" + milliseconds + ".png";
            using (FileStream bi_FileStream = new FileStream(filePath, FileMode.Create))
            {
                BitmapEncoder encoder = new PngBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(_imageBI));
                encoder.Save(bi_FileStream);
                //Console.WriteLine("saved..");
                bi_FileStream.Close();
                bi_FileStream.Dispose();
            }

            _bodyIndexFrame.Dispose();
        }
    }
}
