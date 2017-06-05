using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Windows.Shapes;
using System.Windows.Controls;
using System;

namespace kinect20
{
    public partial class MainWindow : Window
    {
        KinectSensor kinectSensor = null;
        MultiSourceFrameReader multiSourceFrameReader = null;
        Body[] bodies;

        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += MainWindow_Loaded;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensor = KinectSensor.GetDefault();

            if (kinectSensor != null)
            {
                kinectSensor.Open();
            }

            multiSourceFrameReader = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body |
                                                                             FrameSourceTypes.Color);

            multiSourceFrameReader.MultiSourceFrameArrived += MultiSourceFrameReader_MultiSourceFrameArrived;
        }

        private void MultiSourceFrameReader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            using (ColorFrame multiSourceFrame = reference.ColorFrameReference.AcquireFrame())
            using (BodyFrame bodyFrame = reference.BodyFrameReference.AcquireFrame())
            {
                if (multiSourceFrame != null && bodyFrame != null)
                {
                    kinectImage.Source = ToBitmap(multiSourceFrame);
                    bodies = new Body[bodyFrame.BodyFrameSource.BodyCount];
                    bodyFrame.GetAndRefreshBodyData(bodies);
                    elbowRightCanvas.Children.Clear();
                    handRightCanvas.Children.Clear();
                    shoulderRightCanvas.Children.Clear();

                    foreach (Body body in bodies)
                    {
                        if(body.IsTracked)
                        {
                            Joint shoulderRight = body.Joints[JointType.ShoulderRight];
                            drawPointToTracking(shoulderRight, shoulderRightCanvas);

                            Joint elbowRight = body.Joints[JointType.ElbowRight];
                            drawPointToTracking(elbowRight, elbowRightCanvas);

                            Joint handRight = body.Joints[JointType.HandRight];
                            drawPointToTracking(handRight, handRightCanvas);

                            calculateAngle(handRight, elbowRight, shoulderRight);
                        }
                    }
                }
            }
        }

        private ImageSource ToBitmap(ColorFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;

            byte[] pixels = new byte[width * height * ((PixelFormats.Bgr32.BitsPerPixel + 7) / 8)];

            if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                frame.CopyRawFrameDataToArray(pixels);
            }
            else
            {
                frame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
            }

            int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
        }

        private void drawPointToTracking(Joint bodyJoint, Canvas canvasPoint)
        {
            if (bodyJoint.TrackingState == TrackingState.Tracked)
            {
                DepthSpacePoint depthSpacePoint = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(bodyJoint.Position);
                Ellipse circle = new Ellipse() { Width = 25, Height = 25, Fill = new SolidColorBrush(Color.FromArgb(255, 255, 0, 0)) };
                canvasPoint.Children.Add(circle);
                Canvas.SetLeft(circle, depthSpacePoint.X);
                Canvas.SetTop(circle, depthSpacePoint.Y);
            }
        }

        private void calculateAngle(Joint hand, Joint elbow, Joint shoulder)
        {
            DepthSpacePoint depthSpacePointHand = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(hand.Position);
            DepthSpacePoint depthSpacePointElbow = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(elbow.Position);
            DepthSpacePoint depthSpacePointShoulder = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(shoulder.Position);

            double[] elbowHand = { (depthSpacePointHand.X - depthSpacePointElbow.X),
                                    (depthSpacePointHand.Y - depthSpacePointElbow.Y)};

            double[] elbowShoulder = { (depthSpacePointShoulder.X - depthSpacePointElbow.X),
                                    (depthSpacePointShoulder.Y - depthSpacePointElbow.Y)};

            double elbowHandMagnitude = Math.Sqrt(elbowHand[0] * elbowHand[0] + elbowHand[1] * elbowHand[1]);
            double elbowShoulderMagnitude = Math.Sqrt(elbowShoulder[0] * elbowShoulder[0]
                + elbowShoulder[1] * elbowShoulder[1]);

            //normalization
            elbowHand[0] /= elbowHandMagnitude;
            elbowHand[1] /= elbowHandMagnitude;

            elbowShoulder[0] /= elbowShoulderMagnitude;
            elbowShoulder[1] /= elbowShoulderMagnitude;

            double result = elbowHand[0] * elbowShoulder[0] + elbowHand[1] * elbowShoulder[1];
            double angle = Math.Acos(result) * 180 / Math.PI;
            label.Content = angle.ToString();
        }
    }
}
