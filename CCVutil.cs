using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using Cognex.VisionPro;
using Cognex.VisionPro.ImageProcessing;
using Cognex.VisionPro.ImageFile;
using System.Xml.Linq;

namespace VisionSystem
{
    public static class CCVutil
    {
        //static double fx;
        //static double fy;

        public static Mat BitmapToMat(Bitmap bitmap)
        {
            try
            {
                using (Mat mat = BitmapConverter.ToMat(bitmap))
                {

                    return mat.Clone();

                }
            }
            catch
            {
                return null;
            }

        }

        public static Bitmap MatToBitmap(Mat mat)
        {
            try
            {
                using (Bitmap bitmap = BitmapConverter.ToBitmap(mat))
                {
                    return (Bitmap)bitmap.Clone();
                }
            }
            catch
            {
                return null;
            }
        }

        public static ICogImage CVCrop(ICogImage cogImage, int x, int y, int w, int h)
        {
            try
            {
                using (Bitmap bitmap = cogImage.ToBitmap())
                using (Mat mat = new Mat(BitmapConverter.ToMat(bitmap)))
                using (Mat CropMat = mat.SubMat(new Rect(x, y, w, h)))
                {
                    using (Bitmap bitmap1 = new Bitmap(BitmapConverter.ToBitmap(CropMat)))
                    {


                        return (ICogImage)bitmap1.Clone();
                    }
                }
            }
            catch
            {
                return null;
            }
        }

        public static ICogImage CVCrop(ICogImage cogImage, Rect rect)
        {
            try
            {
                using (Bitmap bitmap = cogImage.ToBitmap())
                using (Mat mat = new Mat(BitmapConverter.ToMat(bitmap)))
                using (Mat CropMat = mat.SubMat(rect))
                {
                    using (Bitmap bitmap1 = new Bitmap(BitmapConverter.ToBitmap(CropMat)))
                    {


                        return (ICogImage)bitmap1.Clone();
                    }
                }
            }
            catch
            {
                return null;
            }
        }



        // -1 : XY, 0: X, 1: Y
        public static ICogImage CVFlip(ICogImage cogImage, int mode)
        {
            try
            {
                using (Bitmap bitmap = cogImage.ToBitmap())
                using (Mat mat = new Mat(BitmapConverter.ToMat(bitmap)))
                using (Mat FlipMat = new Mat())
                {

                    Cv2.Flip(mat, FlipMat, (FlipMode)mode);

                    using (Bitmap bitmap1 = new Bitmap(BitmapConverter.ToBitmap(FlipMat)))
                    {


                        return (ICogImage)bitmap1.Clone();
                    }


                }
            }
            catch
            {
                return null;
            }
        }


        public static ICogImage CVRotate(ICogImage cogImage, int angle)
        {
            try
            {
                using (Bitmap bitmap = cogImage.ToBitmap())
                using (Mat mat = BitmapConverter.ToMat(bitmap))
                using (Mat RotateMat = new Mat(mat.Size(), MatType.CV_8UC3))
                using (Mat matrix = Cv2.GetRotationMatrix2D(new Point2f(mat.Width / 2, mat.Height / 2), angle, 1))
                {

                    Cv2.WarpAffine(mat, RotateMat, matrix, mat.Size(), InterpolationFlags.Linear);

                    using (Bitmap bitmap1 = new Bitmap(BitmapConverter.ToBitmap(matrix)))
                    {
                        return (ICogImage)bitmap1.Clone();
                    }


                }


            }
            catch
            {

                return null;
            }
        }


        public static ICogImage CogToMatBinThreshold(ICogImage cogimage, int thtreVal)
        {

            try
            {
                using (Bitmap bitmap = cogimage.ToBitmap())
                using (Mat mat = BitmapConverter.ToMat(bitmap))
                using (Mat imgBin = new Mat())
                {
                    Cv2.Threshold(mat, imgBin, thtreVal, 255, ThresholdTypes.Binary);

                    cogimage = new CogImage8Grey(OpenCvSharp.Extensions.BitmapConverter.ToBitmap(imgBin));

                    return cogimage;
                }



            }
            catch
            {
                return null;
            }

        }

        public static ICogImage SubImage(ICogImage cogImage, int x1, int y1, int x2, int y2)
        {
            try
            {
                using (Mat mat = BitmapToMat(cogImage.ToBitmap()))
                using (Mat Submat = mat.SubMat(new Rect(x1, y1, x2, y2)))
                {

                    cogImage = new CogImage8Grey(Submat.ToBitmap());

                    return cogImage;
                }



            }
            catch
            {
                return null;
            }
        }

        public static ICogImage DrawLine(ICogImage image, int X1, int Y1, int X2, int Y2, int thick, int Color)
        {
            try
            {
                ICogImage cogImamge = null;

                using (Mat mat = BitmapToMat(image.ToBitmap()))
                {

                    Cv2.Line(mat, new OpenCvSharp.Point(X1, Y1), new OpenCvSharp.Point(X2, Y2), Color, thick, LineTypes.AntiAlias);

                    using (Bitmap bitmap = new Bitmap(BitmapConverter.ToBitmap(mat)))
                    {
                        cogImamge = new CogImage24PlanarColor(bitmap);

                        return cogImamge;
                    }


                }


            }
            catch
            {
                return null;
            }
        }

        public static ICogImage DrawCircle(ICogImage image, int CenterX, int CenterY, int Radius, int thick, int Color)
        {
            try
            {
                ICogImage cogImamge = null;

                using (Mat mat = BitmapToMat(image.ToBitmap()))
                {

                    Cv2.Circle(mat, new OpenCvSharp.Point(CenterX, CenterY), Radius, Color, thick, LineTypes.AntiAlias);

                    using (Bitmap bitmap = new Bitmap(BitmapConverter.ToBitmap(mat)))
                    {
                        cogImamge = new CogImage24PlanarColor(bitmap);

                        return cogImamge;
                    }


                }
            }
            catch
            {
                return null;
            }
        }

        public static ICogImage DrawRectAngle(ICogImage image, int X1, int Y1, int X2, int Y2, int thick, int Color)
        {
            try
            {
                ICogImage cogImamge = null;

                using (Mat mat = BitmapToMat(image.ToBitmap()))
                {

                    Cv2.Rectangle(mat, new OpenCvSharp.Point(X1, Y1), new OpenCvSharp.Point(X2, Y2), Color, thick, LineTypes.AntiAlias);

                    using (Bitmap bitmap = new Bitmap(BitmapConverter.ToBitmap(mat)))
                    {
                        cogImamge = new CogImage24PlanarColor(bitmap);

                        return cogImamge;
                    }


                }


            }
            catch
            {
                return null;
            }
        }

        public static ICogImage DrawEllipse(ICogImage image, int CenterX, int CenterY, int SizeWidth, int SizeHeight, double angle, double startAngle, double endAngle, int Color, int thickness)
        {
            try
            {
                ICogImage cogImamge = null;

                using (Mat mat = BitmapToMat(image.ToBitmap()))
                {

                    Cv2.Ellipse(mat, new OpenCvSharp.Point(CenterX, CenterY), new OpenCvSharp.Size(SizeWidth, SizeHeight), angle, startAngle, endAngle, Color, thickness, LineTypes.AntiAlias);

                    using (Bitmap bitmap = new Bitmap(BitmapConverter.ToBitmap(mat)))
                    {

                        cogImamge = new CogImage24PlanarColor(bitmap);


                        return cogImamge;
                    }



                }


            }
            catch
            {
                return null;
            }
        }

        public static ICogImage CvPutText(ICogImage image, string text, int CenterX, int CenterY, double FontSize, int Color, int thick)
        {
            try
            {
                ICogImage cogImamge = null;

                using (Mat mat = BitmapToMat(image.ToBitmap()))
                {

                    Cv2.PutText(mat, text, new OpenCvSharp.Point(CenterX, CenterY), HersheyFonts.HersheyComplex, FontSize, Color, thick, LineTypes.AntiAlias);

                    return cogImamge;
                }


            }
            catch
            {
                return null;
            }
        }

        public static Bitmap CVRotateBitmap(Bitmap bitmap, int angle)
        {
            try
            {

                using (Mat mat = BitmapConverter.ToMat(bitmap))
                using (Mat RotateMat = new Mat(mat.Size(), MatType.CV_8UC3))
                using (Mat matrix = Cv2.GetRotationMatrix2D(new Point2f(mat.Width / 2, mat.Height / 2), angle, 1))
                {

                    Cv2.WarpAffine(mat, RotateMat, matrix, mat.Size(), InterpolationFlags.Linear);

                    Bitmap bitmap1 = new Bitmap(BitmapConverter.ToBitmap(RotateMat));

                    return bitmap1;



                }


            }
            catch
            {

                return null;
            }
        }


        public static Bitmap CVFlipBitmap(Bitmap bitmap, int mode)
        {
            try
            {

                using (Mat mat = BitmapConverter.ToMat(bitmap))
                using (Mat FlipMat = new Mat(mat.Size(), MatType.CV_8UC3))
                {

                    switch (mode)
                    {
                        case 0:

                            return bitmap;

                            break;

                        case 1:

                            Cv2.Flip(mat, FlipMat, FlipMode.X);




                            break;
                        case 2:

                            Cv2.Flip(mat, FlipMat, FlipMode.Y);






                            break;

                        case 3:

                            Cv2.Flip(mat, FlipMat, FlipMode.XY);


                            break;
                    }

                    return new Bitmap(BitmapConverter.ToBitmap(FlipMat));
                }
            }
            catch
            {
                return null;
            }
        }
        public static (double, double) CheckBoardCalibration(string path, int rowCount, int colCount, double squareSize)
        {
            double fx = 0.0, fy = 0.0;
            double pxPerMmX = 0.0, pxPerMmY = 0.0;

            try
            {
                


                using (Mat mat = Cv2.ImRead(path))
                using (Mat matGray = mat.CvtColor(ColorConversionCodes.BGR2GRAY))
                {
                    List<Point3f> objPoints = new List<Point3f>();
                    List<Point2f> imgPoints = new List<Point2f>();

                    OpenCvSharp.Size patternSize = new OpenCvSharp.Size(rowCount, colCount);

                    // 체커보드의 3D 포인트
                    for (int i = 0; i < colCount; i++)
                    {
                        for (int j = 0; j < rowCount; j++)
                        {
                            objPoints.Add(new Point3f(j, i, 0));
                        }
                    }

                    Point2f[] corners;
                    // 체커보드 코너 찾기
                    bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

                    if (found)
                    {
                        Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
                        Cv2.DrawChessboardCorners(matGray, patternSize, corners, found);

                        Cv2.ImShow("dasdad", matGray);
                        Cv2.WaitKey(0);

                        // 캘리브레이션을 위한 포인트 준비
                        List<Mat> objectPoints = new List<Mat> { new Mat(objPoints.Count, 1, MatType.CV_32FC3, objPoints.ToArray()) };
                        List<Mat> imagePoints = new List<Mat> { new Mat(corners.Length, 1, MatType.CV_32FC2, corners) };

                        // 카메라 캘리브레이션
                        //Mat cameraMatrix = new Mat(3, 3, MatType.CV_64FC1);
                        //Mat distCoeffs = new Mat(8, 1, MatType.CV_64FC1);
                        Mat cameraMatrix = new Mat();
                        Mat distCoeffs = new Mat();
                        Mat[] rvecs, tvecs;

                        double rms = Cv2.CalibrateCamera(
                            objectPoints,
                            imagePoints,
                            matGray.Size(),
                            cameraMatrix,
                            distCoeffs,
                            out rvecs,
                            out tvecs
                        );

                        //Console.WriteLine($"RMS error: {rms}");
                        //Console.WriteLine("Camera Matrix:");
                        //Console.WriteLine(cameraMatrix.Dump());
                        //Console.WriteLine("Distortion Coefficients:");
                        //Console.WriteLine(distCoeffs.Dump());

                        // 분해능 계산 (픽셀 크기)
                        fx = cameraMatrix.At<double>(0, 0);
                        fy = cameraMatrix.At<double>(1, 1);

                        // 1픽셀당 mm 계산
                        pxPerMmX = fx / squareSize;
                        pxPerMmY = fy / squareSize;

                        //Console.WriteLine($"x resolution: {fx} pixels");
                        //Console.WriteLine($"y resolution: {fy} pixels");
                        //Console.WriteLine($"1 pixel = {1 / pxPerMmX} mm (X axis)");
                        //Console.WriteLine($"1 pixel = {1 / pxPerMmY} mm (Y axis)");

                    }
                    else
                    {
                        //Console.WriteLine("Checkerboard corners not found in the image.");
                    }
                }
            }
            catch (Exception ex)
            {
                //Console.WriteLine($"Exception occurred: {ex.Message}");
                return (9.9, 9.9);
            }

            return (Math.Round(1 / pxPerMmX, 2), Math.Round(1 / pxPerMmY, 2));




        }

        public static (CogImage8Grey, double, double) CheckBoardCal(string path, int rowCount, int colCount, double squareSize)
        {
            double fx = 0.0, fy = 0.0;
            double pxPerMmX = 0.0, pxPerMmY = 0.0;
            CogImage8Grey calImage = null;

            try
            {



                using (Mat mat = Cv2.ImRead(path))
                using (Mat matGray = mat.CvtColor(ColorConversionCodes.BGR2GRAY))
                {
                    List<Point3f> objPoints = new List<Point3f>();
                    List<Point2f> imgPoints = new List<Point2f>();

                    OpenCvSharp.Size patternSize = new OpenCvSharp.Size(rowCount, colCount);

                    // 체커보드의 3D 포인트
                    for (int i = 0; i < colCount; i++)
                    {
                        for (int j = 0; j < rowCount; j++)
                        {
                            //objPoints.Add(new Point3f(j, i, 0));
                            objPoints.Add(new Point3f(j * (float)squareSize, i * (float)squareSize, 0));
                        }
                    }

                    Point2f[] corners;
                    // 체커보드 코너 찾기
                    bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

                    if (found)
                    {
                        Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
                        Cv2.DrawChessboardCorners(matGray, patternSize, corners, found);

                        

                        // 캘리브레이션을 위한 포인트 준비
                        List<Mat> objectPoints = new List<Mat> { new Mat(objPoints.Count, 1, MatType.CV_32FC3, objPoints.ToArray()) };
                        List<Mat> imagePoints = new List<Mat> { new Mat(corners.Length, 1, MatType.CV_32FC2, corners) };

                        // 카메라 캘리브레이션
                        //Mat cameraMatrix = new Mat(3, 3, MatType.CV_64FC1);
                        //Mat distCoeffs = new Mat(8, 1, MatType.CV_64FC1);
                        Mat cameraMatrix = new Mat();
                        Mat distCoeffs = new Mat();
                        Mat[] rvecs, tvecs;

                        double rms = Cv2.CalibrateCamera(
                            objectPoints,
                            imagePoints,
                            matGray.Size(),
                            cameraMatrix,
                            distCoeffs,
                            out rvecs,
                            out tvecs
                        );

                        //Console.WriteLine($"RMS error: {rms}");
                        //Console.WriteLine("Camera Matrix:");
                        //Console.WriteLine(cameraMatrix.Dump());
                        //Console.WriteLine("Distortion Coefficients:");
                        //Console.WriteLine(distCoeffs.Dump());

                        // 분해능 계산 (픽셀 크기)
                        fx = cameraMatrix.At<double>(0, 0);
                        fy = cameraMatrix.At<double>(1, 1);

                        // 1mm당 픽셀 수 계산
                        pxPerMmX = fx / squareSize;
                        pxPerMmY = fy / squareSize;

                        // 1픽셀당 mm 계산
                        double mmPerPxX = 1 / pxPerMmX;
                        double mmPerPxY = 1 / pxPerMmY;

                        //Console.WriteLine($"x resolution: {fx} pixels");
                        //Console.WriteLine($"y resolution: {fy} pixels");
                        //Console.WriteLine($"1 pixel = {1 / pxPerMmX} mm (X axis)");
                        //Console.WriteLine($"1 pixel = {1 / pxPerMmY} mm (Y axis)");

                        using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(matGray))
                        {
                            calImage = new CogImage8Grey(tmp);
                        }

                        return (calImage, Math.Round(mmPerPxX, 5), Math.Round(mmPerPxY, 5));

                    }
                    else
                    {
                        return (null, 9.9, 9.9);
                        //Console.WriteLine("Checkerboard corners not found in the image.");
                    }
                }
            }
            catch (Exception ex)
            {
                //Console.WriteLine($"Exception occurred: {ex.Message}");
                return (null, 9.9, 9.9);
            }

            




        }

        //public static (CogImage8Grey, double, double) CheckBoardCal(string[] imagePaths, int rowCount, int colCount, double squareSize)
        //{
        //    double fx = 0.0, fy = 0.0;
        //    double pxPerMmX = 0.0, pxPerMmY = 0.0;
        //    CogImage8Grey calImage = null;

        //    try
        //    {
        //        List<Point3f> objPoints = new List<Point3f>();
        //        List<List<Point2f>> imgPointsList = new List<List<Point2f>>();
        //        List<List<Point3f>> objPointsList = new List<List<Point3f>>();

        //        OpenCvSharp.Size patternSize = new OpenCvSharp.Size(rowCount, colCount);

        //        // 체커보드의 3D 포인트
        //        for (int i = 0; i < colCount; i++)
        //        {
        //            for (int j = 0; j < rowCount; j++)
        //            {
        //                objPoints.Add(new Point3f(j * (float)squareSize, i * (float)squareSize, 0));
        //            }
        //        }

        //        OpenCvSharp.Size imageSize = new OpenCvSharp.Size();

        //        foreach (string path in imagePaths)
        //        {
        //            using (Mat mat = Cv2.ImRead(path))
        //            using (Mat matGray = mat.CvtColor(ColorConversionCodes.BGR2GRAY))
        //            {
        //                imageSize = mat.Size();

        //                Point2f[] corners;
        //                // 체커보드 코너 찾기
        //                bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

        //                if (found)
        //                {
        //                    Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
        //                    Cv2.DrawChessboardCorners(matGray, patternSize, corners, found);

        //                    imgPointsList.Add(new List<Point2f>(corners));
        //                    objPointsList.Add(new List<Point3f>(objPoints));

        //                    // 마지막 이미지로 calImage 설정
        //                    using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(matGray))
        //                    {
        //                        calImage = new CogImage8Grey(tmp);
        //                    }
        //                }
        //            }
        //        }

        //        if (imgPointsList.Count > 0)
        //        {
        //            Mat cameraMatrix = new Mat();
        //            Mat distCoeffs = new Mat();
        //            Mat[] rvecs, tvecs;

        //            double rms = Cv2.CalibrateCamera(
        //                objPointsList.Select(op => new Mat(op.Count, 1, MatType.CV_32FC3, op.ToArray())).ToList(),
        //                imgPointsList.Select(ip => new Mat(ip.Count, 1, MatType.CV_32FC2, ip.ToArray())).ToList(),
        //                imageSize,
        //                cameraMatrix,
        //                distCoeffs,
        //                out rvecs,
        //                out tvecs
        //            );

        //            // 분해능 계산 (픽셀 크기)
        //            fx = cameraMatrix.At<double>(0, 0);
        //            fy = cameraMatrix.At<double>(1, 1);

        //            // 1mm당 픽셀 수 계산
        //            pxPerMmX = fx / squareSize;
        //            pxPerMmY = fy / squareSize;

        //            // 1픽셀당 mm 계산
        //            double mmPerPxX = squareSize / fx;
        //            double mmPerPxY = squareSize / fy;

        //            // 이미지 보정
        //            foreach (string path in imagePaths)
        //            {
        //                using (Mat mat = Cv2.ImRead(path))
        //                {
        //                    Mat undistorted = new Mat();
        //                    Cv2.Undistort(mat, undistorted, cameraMatrix, distCoeffs);
        //                    // 보정된 이미지에서 다시 코너를 검출하여 거리 측정
        //                    using (Mat matGray = undistorted.CvtColor(ColorConversionCodes.BGR2GRAY))
        //                    {
        //                        Point2f[] corners;
        //                        bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

        //                        if (found)
        //                        {
        //                            Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
        //                            //double distance = Cv2.Norm(corners[0] - corners[1]);
        //                            double distance = Math.Sqrt(Math.Pow(corners[0].X - corners[1].X, 2) + Math.Pow(corners[0].Y - corners[1].Y, 2));
        //                            double measuredDistance = distance * mmPerPxX;
        //                            Console.WriteLine($"Measured distance: {measuredDistance} mm");
        //                        }
        //                    }
        //                }
        //            }


        //            return (calImage, Math.Round(mmPerPxX, 5), Math.Round(mmPerPxY, 5));
        //        }
        //        else
        //        {
        //            return (null, 9.9, 9.9); // 체커보드 코너를 찾지 못한 경우
        //        }


        //    }
        //    catch (Exception ex)
        //    {
        //        //Console.WriteLine($"Exception occurred: {ex.Message}");
        //        return (null, 9.9, 9.9);
        //    }






        //}

        public static (CogImage8Grey, double, double) CheckBoardCal(string[] imagePaths, int rowCount, int colCount, double squareSize)
        {
            double fx = 0.0, fy = 0.0;
            double pxPerMmX = 0.0, pxPerMmY = 0.0;
            CogImage8Grey calImage = null;

            try
            {
                List<Point3f> objPoints = new List<Point3f>();
                List<List<Point2f>> imgPointsList = new List<List<Point2f>>();
                List<List<Point3f>> objPointsList = new List<List<Point3f>>();

                OpenCvSharp.Size patternSize = new OpenCvSharp.Size(rowCount, colCount);

                // 체커보드의 3D 포인트
                for (int i = 0; i < colCount; i++)
                {
                    for (int j = 0; j < rowCount; j++)
                    {
                        objPoints.Add(new Point3f(j * (float)squareSize, i * (float)squareSize, 0));
                    }
                }

                OpenCvSharp.Size imageSize = new OpenCvSharp.Size();

                foreach (string path in imagePaths)
                {
                    using (Mat mat = Cv2.ImRead(path))
                    using (Mat matGray = mat.CvtColor(ColorConversionCodes.BGR2GRAY))
                    {
                        imageSize = mat.Size();

                        Point2f[] corners;
                        // 체커보드 코너 찾기
                        bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

                        double pixelDistanceX = Math.Sqrt(Math.Pow(corners[0].X - corners[1].X, 2) + Math.Pow(corners[0].Y - corners[1].Y, 2));
                        //Console.WriteLine($"Before Cal Pixel Distance between corner 1 and corner 2: {pixelDistanceX}");
                        double pixelDistanceY = Math.Sqrt(Math.Pow(corners[0].X - corners[rowCount].X, 2) + Math.Pow(corners[0].Y - corners[rowCount].Y, 2));
                        //Console.WriteLine($"Before Cal Pixel Distance between corner 1 and corner 2: {pixelDistanceY}");

                        if (found)
                        {
                            Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
                            Cv2.DrawChessboardCorners(matGray, patternSize, corners, found);

                            imgPointsList.Add(new List<Point2f>(corners));
                            objPointsList.Add(new List<Point3f>(objPoints));

                            // 마지막 이미지로 calImage 설정
                            using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(matGray))
                            {
                                calImage = new CogImage8Grey(tmp);
                            }
                        }
                    }
                }

                if (imgPointsList.Count > 0)
                {
                    List<Point2f> cornerstmp = imgPointsList[0];
                    Mat cameraMatrix = new Mat();
                    Mat distCoeffs = new Mat();
                    Mat[] rvecs, tvecs;

                    double rms = Cv2.CalibrateCamera(
                        objPointsList.Select(op => new Mat(op.Count, 1, MatType.CV_32FC3, op.ToArray())).ToList(),
                        imgPointsList.Select(ip => new Mat(ip.Count, 1, MatType.CV_32FC2, ip.ToArray())).ToList(),
                        imageSize,
                        cameraMatrix, //캘리브레이션 결과로 얻은 카메라 매트릭스
                        distCoeffs, //캘리브레이션 결과로 얻은 왜곡 계수
                        out rvecs, //캘리브레이션 결과로 얻은 각 이미지에 대한 회전 벡터 배열
                        out tvecs //캘리브레이션 결과로 얻은 각이미지에 대한 평행 이동 벡터 배열
                    );

                    // 카메라 매트릭스 및 왜곡 계수 출력
                    //Console.WriteLine("Camera Matrix:");
                    //for (int i = 0; i < cameraMatrix.Rows; i++)
                    //{
                    //    for (int j = 0; j < cameraMatrix.Cols; j++)
                    //    {
                    //        Console.Write(cameraMatrix.At<double>(i, j) + " ");
                    //    }
                    //    Console.WriteLine();
                    //}

                    // 분해능 계산 (픽셀 크기) 이건 카메라 내부 파라미터 분해능이라 이미지에 적용하면 안됌
                    fx = cameraMatrix.At<double>(0, 0);
                    fy = cameraMatrix.At<double>(1, 1);


                    //1픽셀당 mm 계산 //이것도 카메라 내부파라미터 기준
                    pxPerMmX = squareSize / fx;
                    pxPerMmY = squareSize / fy;

                   

                    // 보정된 이미지에서 거리 측정
                    foreach (string path in imagePaths)
                    {
                        using (Mat mat = Cv2.ImRead(path))
                        using (Mat undistorted = new Mat())
                        {
                            
                            Cv2.Undistort(mat, undistorted, cameraMatrix, distCoeffs);

                            using (Mat matGray = undistorted.CvtColor(ColorConversionCodes.BGR2GRAY))
                            {
                                Point2f[] corners;
                                bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

                                if (found)
                                {
                                    Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
                                    double pixelDistanceX = Math.Sqrt(Math.Pow(corners[0].X - corners[1].X, 2) + Math.Pow(corners[0].Y - corners[1].Y, 2));
                                    Console.WriteLine($"After Cal Pixel Distance between corner 1 and corner 2: {pixelDistanceX}");
                                    double pixelDistanceY = Math.Sqrt(Math.Pow(corners[0].X - corners[rowCount].X, 2) + Math.Pow(corners[0].Y - corners[rowCount].Y, 2));
                                    Console.WriteLine($"After Cal Pixel Distance between corner 1 and corner 2: {pixelDistanceY}");

                                    pxPerMmX = squareSize / pixelDistanceX;
                                    pxPerMmY = squareSize / pixelDistanceY;

                                    // 실제 거리 계산
                                    double actualDistanceX = pixelDistanceX * pxPerMmX;
                                    //Console.WriteLine($"After Cal Actual Distance between corner 1 and corner 2: {actualDistanceX} mm");
                                    double actualDistanceY = pixelDistanceY * pxPerMmY;
                                    //Console.WriteLine($"After Cal Actual Distance between corner 1 and corner 2: {actualDistanceY} mm");

                                    // 두 포인트 간의 선을 이미지에 그리기
                                    Cv2.Line(undistorted, (int)corners[0].X, (int)corners[0].Y, (int)corners[1].X, (int)corners[1].Y, Scalar.Red, 2);
                                    Cv2.PutText(undistorted, $"Distance: {actualDistanceX} mm", new OpenCvSharp.Point((int)(corners[0].X + corners[1].X) / 2, ((int)(corners[0].Y + corners[1].Y) / 2) + 10),
                                        HersheyFonts.HersheySimplex, 0.5, Scalar.Red);

                                    // 두 포인트 간의 선을 이미지에 그리기
                                    Cv2.Line(undistorted, (int)corners[0].X, (int)corners[0].Y, (int)corners[rowCount].X, (int)corners[rowCount].Y, Scalar.Red, 2);
                                    Cv2.PutText(undistorted, $"Distance: {actualDistanceY} mm", new OpenCvSharp.Point(((int)(corners[0].X + corners[rowCount].X) / 2) + 10, (int)(corners[0].Y + corners[rowCount].Y) / 2),
                                        HersheyFonts.HersheySimplex, 0.5, Scalar.Red);

                                    // 마지막 이미지로 calImage 설정
                                    using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(undistorted))
                                    {
                                        calImage = new CogImage8Grey(tmp);
                                    }
                                }
                            }
                        }
                    }

                    return (calImage, Math.Round(pxPerMmX, 5), Math.Round(pxPerMmY, 5));
                }
                else
                {
                    return (null, 9.9, 9.9); // 체커보드 코너를 찾지 못한 경우
                }
            }
            catch (Exception ex)
            {
                //Console.WriteLine($"Exception occurred: {ex.Message}");
                return (null, 9.9, 9.9);
            }
        }

        public static (CogImage8Grey, double, double, Mat, Mat) CheckBoardCalMat(string[] imagePaths, int rowCount, int colCount, double squareSize)
        {
            double fx = 0.0, fy = 0.0;
            double pxPerMmX = 0.0, pxPerMmY = 0.0;
            CogImage8Grey calImage = null;
            

            try
            {
                List<Point3f> objPoints = new List<Point3f>();
                List<List<Point2f>> imgPointsList = new List<List<Point2f>>();
                List<List<Point3f>> objPointsList = new List<List<Point3f>>();

                OpenCvSharp.Size patternSize = new OpenCvSharp.Size(rowCount, colCount);

                // 체커보드의 3D 포인트
                for (int i = 0; i < colCount; i++)
                {
                    for (int j = 0; j < rowCount; j++)
                    {
                        objPoints.Add(new Point3f(j * (float)squareSize, i * (float)squareSize, 0));
                    }
                }

                OpenCvSharp.Size imageSize = new OpenCvSharp.Size();

                foreach (string path in imagePaths)
                {
                    using (Mat mat = Cv2.ImRead(path))
                    using (Mat matGray = mat.CvtColor(ColorConversionCodes.BGR2GRAY))
                    {
                        imageSize = mat.Size();

                        Point2f[] corners;
                        // 체커보드 코너 찾기
                        bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

                        double pixelDistanceX = Math.Sqrt(Math.Pow(corners[0].X - corners[1].X, 2) + Math.Pow(corners[0].Y - corners[1].Y, 2));
                        //Console.WriteLine($"Before Cal Pixel Distance between corner 1 and corner 2: {pixelDistanceX}");
                        double pixelDistanceY = Math.Sqrt(Math.Pow(corners[0].X - corners[rowCount].X, 2) + Math.Pow(corners[0].Y - corners[rowCount].Y, 2));
                        //Console.WriteLine($"Before Cal Pixel Distance between corner 1 and corner 2: {pixelDistanceY}");

                        if (found)
                        {
                            Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
                            Cv2.DrawChessboardCorners(matGray, patternSize, corners, found);

                            imgPointsList.Add(new List<Point2f>(corners));
                            objPointsList.Add(new List<Point3f>(objPoints));

                            // 마지막 이미지로 calImage 설정
                            using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(matGray))
                            {
                                calImage = new CogImage8Grey(tmp);
                            }
                        }
                    }
                }

                if (imgPointsList.Count > 0)
                {
                    List<Point2f> cornerstmp = imgPointsList[0];
                    Mat cameraMatrix = new Mat();
                    Mat distCoeffs = new Mat();
                    Mat[] rvecs, tvecs;

                    double rms = Cv2.CalibrateCamera(
                        objPointsList.Select(op => new Mat(op.Count, 1, MatType.CV_32FC3, op.ToArray())).ToList(),
                        imgPointsList.Select(ip => new Mat(ip.Count, 1, MatType.CV_32FC2, ip.ToArray())).ToList(),
                        imageSize,
                        cameraMatrix, //캘리브레이션 결과로 얻은 카메라 매트릭스
                        distCoeffs, //캘리브레이션 결과로 얻은 왜곡 계수
                        out rvecs, //캘리브레이션 결과로 얻은 각 이미지에 대한 회전 벡터 배열
                        out tvecs //캘리브레이션 결과로 얻은 각이미지에 대한 평행 이동 벡터 배열
                    );

                    // 카메라 매트릭스 및 왜곡 계수 출력
                    //Console.WriteLine("Camera Matrix:");
                    //for (int i = 0; i < cameraMatrix.Rows; i++)
                    //{
                    //    for (int j = 0; j < cameraMatrix.Cols; j++)
                    //    {
                    //        Console.Write(cameraMatrix.At<double>(i, j) + " ");
                    //    }
                    //    Console.WriteLine();
                    //}

                    // 분해능 계산 (픽셀 크기) 이건 카메라 내부 파라미터 분해능이라 이미지에 적용하면 안됌
                    fx = cameraMatrix.At<double>(0, 0);
                    fy = cameraMatrix.At<double>(1, 1);


                    //1픽셀당 mm 계산 //이것도 카메라 내부파라미터 기준
                    pxPerMmX = squareSize / fx;
                    pxPerMmY = squareSize / fy;



                    // 보정된 이미지에서 거리 측정
                    foreach (string path in imagePaths)
                    {
                        using (Mat mat = Cv2.ImRead(path))
                        using (Mat undistorted = new Mat())
                        {

                            Cv2.Undistort(mat, undistorted, cameraMatrix, distCoeffs);

                            using (Mat matGray = undistorted.CvtColor(ColorConversionCodes.BGR2GRAY))
                            {
                                Point2f[] corners;
                                bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

                                if (found)
                                {
                                    Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
                                    double pixelDistanceX = Math.Sqrt(Math.Pow(corners[0].X - corners[1].X, 2) + Math.Pow(corners[0].Y - corners[1].Y, 2));
                                    Console.WriteLine($"After Cal Pixel Distance between corner 1 and corner 2: {pixelDistanceX}");
                                    double pixelDistanceY = Math.Sqrt(Math.Pow(corners[0].X - corners[rowCount].X, 2) + Math.Pow(corners[0].Y - corners[rowCount].Y, 2));
                                    Console.WriteLine($"After Cal Pixel Distance between corner 1 and corner 2: {pixelDistanceY}");

                                    pxPerMmX = squareSize / pixelDistanceX;
                                    pxPerMmY = squareSize / pixelDistanceY;

                                    // 실제 거리 계산
                                    double actualDistanceX = pixelDistanceX * pxPerMmX;
                                    //Console.WriteLine($"After Cal Actual Distance between corner 1 and corner 2: {actualDistanceX} mm");
                                    double actualDistanceY = pixelDistanceY * pxPerMmY;
                                    //Console.WriteLine($"After Cal Actual Distance between corner 1 and corner 2: {actualDistanceY} mm");

                                    // 두 포인트 간의 선을 이미지에 그리기
                                    Cv2.Line(undistorted, (int)corners[0].X, (int)corners[0].Y, (int)corners[1].X, (int)corners[1].Y, Scalar.Red, 2);
                                    Cv2.PutText(undistorted, $"Distance: {actualDistanceX} mm", new OpenCvSharp.Point((int)(corners[0].X + corners[1].X) / 2, ((int)(corners[0].Y + corners[1].Y) / 2) + 10),
                                        HersheyFonts.HersheySimplex, 0.5, Scalar.Red);

                                    // 두 포인트 간의 선을 이미지에 그리기
                                    Cv2.Line(undistorted, (int)corners[0].X, (int)corners[0].Y, (int)corners[rowCount].X, (int)corners[rowCount].Y, Scalar.Red, 2);
                                    Cv2.PutText(undistorted, $"Distance: {actualDistanceY} mm", new OpenCvSharp.Point(((int)(corners[0].X + corners[rowCount].X) / 2) + 10, (int)(corners[0].Y + corners[rowCount].Y) / 2),
                                        HersheyFonts.HersheySimplex, 0.5, Scalar.Red);

                                    // 마지막 이미지로 calImage 설정
                                    using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(undistorted))
                                    {
                                        calImage = new CogImage8Grey(tmp);
                                    }
                                }
                            }
                        }
                    }

                    return (calImage, Math.Round(pxPerMmX, 5), Math.Round(pxPerMmY, 5), cameraMatrix, distCoeffs);
                }
                else
                {
                    return (null, 9.9, 9.9, null, null); // 체커보드 코너를 찾지 못한 경우
                }
            }
            catch (Exception ex)
            {
                //Console.WriteLine($"Exception occurred: {ex.Message}");
                return (null, 9.9, 9.9, null, null);
            }
        }

        public static (CogImage8Grey, double, double) CheckBoardCal(string[] imagePaths, int rowCount, int colCount, double squareSize, double sensorWidthMm, double sensorHeightMm, int LensMM)
        {
            double fx = 0.0, fy = 0.0;
            double pxPerMmX = 0.0, pxPerMmY = 0.0;
            CogImage8Grey calImage = null;

            try
            {
                List<Point3f> objPoints = new List<Point3f>();
                List<List<Point2f>> imgPointsList = new List<List<Point2f>>();
                List<List<Point3f>> objPointsList = new List<List<Point3f>>();

                OpenCvSharp.Size patternSize = new OpenCvSharp.Size(rowCount, colCount);

                // 체커보드의 3D 포인트
                for (int i = 0; i < colCount; i++)
                {
                    for (int j = 0; j < rowCount; j++)
                    {
                        objPoints.Add(new Point3f(j * (float)squareSize, i * (float)squareSize, 0));
                    }
                }

                OpenCvSharp.Size imageSize = new OpenCvSharp.Size();

                foreach (string path in imagePaths)
                {
                    using (Mat mat = Cv2.ImRead(path))
                    using (Mat matGray = mat.CvtColor(ColorConversionCodes.BGR2GRAY))
                    {
                        imageSize = mat.Size();

                        Point2f[] corners;
                        // 체커보드 코너 찾기
                        bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

                        if (found)
                        {
                            Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
                            Cv2.DrawChessboardCorners(matGray, patternSize, corners, found);

                            imgPointsList.Add(new List<Point2f>(corners));
                            objPointsList.Add(new List<Point3f>(objPoints));

                            // 마지막 이미지로 calImage 설정
                            using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(matGray))
                            {
                                calImage = new CogImage8Grey(tmp);
                            }
                        }
                    }
                }

                if (imgPointsList.Count > 0)
                {
                    Mat cameraMatrix = new Mat();
                    Mat distCoeffs = new Mat();
                    Mat[] rvecs, tvecs;

                    double rms = Cv2.CalibrateCamera(
                        objPointsList.Select(op => new Mat(op.Count, 1, MatType.CV_32FC3, op.ToArray())).ToList(),
                        imgPointsList.Select(ip => new Mat(ip.Count, 1, MatType.CV_32FC2, ip.ToArray())).ToList(),
                        imageSize,
                        cameraMatrix,
                        distCoeffs,
                        out rvecs,
                        out tvecs
                    );

                    // 분해능 계산 (픽셀 크기)
                    fx = cameraMatrix.At<double>(0, 0);
                    fy = cameraMatrix.At<double>(1, 1);

                    // 센서 크기와 이미지 크기를 사용하여 분해능 계산
                    double sensorWidthPx = imageSize.Width;
                    double sensorHeightPx = imageSize.Height;

                    // 1mm당 픽셀 수 계산
                    pxPerMmX = sensorWidthPx / sensorWidthMm;
                    pxPerMmY = sensorHeightPx / sensorHeightMm;

                    // 1픽셀당 mm 계산
                    double mmPerPxX = sensorWidthMm / sensorWidthPx;
                    double mmPerPxY = sensorHeightMm / sensorHeightPx;

                    // 이미지 보정
                    foreach (string path in imagePaths)
                    {
                        using (Mat mat = Cv2.ImRead(path))
                        {
                            Mat undistorted = new Mat();
                            Cv2.Undistort(mat, undistorted, cameraMatrix, distCoeffs);
                            // 보정된 이미지에서 다시 코너를 검출하여 거리 측정
                            using (Mat matGray = undistorted.CvtColor(ColorConversionCodes.BGR2GRAY))
                            {
                                Point2f[] corners;
                                bool found = Cv2.FindChessboardCorners(matGray, patternSize, out corners);

                                if (found)
                                {
                                    Cv2.CornerSubPix(matGray, corners, new OpenCvSharp.Size(11, 11), new OpenCvSharp.Size(-1, -1), new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.1));
                                    //double distance = Cv2.Norm(corners[0] - corners[1]);
                                    double distance = Math.Sqrt(Math.Pow(corners[0].X - corners[1].X, 2) + Math.Pow(corners[0].Y - corners[1].Y, 2));
                                    double measuredDistance = distance * mmPerPxX;
                                    Console.WriteLine($"Measured distance: {measuredDistance} mm");
                                }
                            }
                        }
                    }


                    return (calImage, Math.Round(mmPerPxX, 5), Math.Round(mmPerPxY, 5));
                }
                else
                {
                    return (null, 9.9, 9.9); // 체커보드 코너를 찾지 못한 경우
                }


            }
            catch (Exception ex)
            {
                //Console.WriteLine($"Exception occurred: {ex.Message}");
                return (null, 9.9, 9.9);
            }






        }

        public static CogImage8Grey UndistortRun(CogImage8Grey image, Mat cameraMatrix, Mat distortionCoefficients)
        {
            try
            {
                CogImage8Grey UndistortImage = null;

                using(Bitmap tmpbitmap = image.ToBitmap())
                using(Mat tmpMat = OpenCvSharp.Extensions.BitmapConverter.ToMat(tmpbitmap))
                using(Mat undistorted = new Mat())
                {

                    Cv2.Undistort(tmpMat, undistorted, cameraMatrix, distortionCoefficients);

                    using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(undistorted))
                    {
                        UndistortImage = new CogImage8Grey(tmp);
                    }


                }


                return UndistortImage;
            }
            catch
            {
                return null;
            }
        }

        public static void UndistortRun(Mat cameraMatrix, Mat distortionCoefficients, ref CogImage8Grey image)
        {
            try
            {
                

                using (Bitmap tmpbitmap = image.ToBitmap())
                using (Mat tmpMat = OpenCvSharp.Extensions.BitmapConverter.ToMat(tmpbitmap))
                using (Mat undistorted = new Mat())
                {

                    Cv2.Undistort(tmpMat, undistorted, cameraMatrix, distortionCoefficients);

                    using (Bitmap tmp = OpenCvSharp.Extensions.BitmapConverter.ToBitmap(undistorted))
                    {
                        image = new CogImage8Grey(tmp);
                    }


                }


                
            }
            catch
            {
                
            }
        }

        public static void SaveCalibration(string filePath, Mat cameraMatrix, Mat distortionCoefficients)
        {
            using (var fs = new FileStorage(filePath, FileStorage.Modes.Write))
            {
                fs.Write("cameraMatrix", cameraMatrix);
                fs.Write("distortionCoefficients", distortionCoefficients);
            }
        }

        public static void LoadCalibration(string filePath, out Mat cameraMatrix, out Mat distortionCoefficients)
        {
            using (var fs = new FileStorage(filePath, FileStorage.Modes.Read))
            {
                cameraMatrix = new Mat();
                distortionCoefficients = new Mat();

                // 데이터를 Mat 객체에 직접 로드
                fs["cameraMatrix"].ReadMat(cameraMatrix);
                fs["distortionCoefficients"].ReadMat(distortionCoefficients);
            }

            // 디버깅 출력 추가
            //Console.WriteLine($"Loaded Camera Matrix: {cameraMatrix.Rows}x{cameraMatrix.Cols}");
            //Console.WriteLine($"Loaded Distortion Coefficients: {distortionCoefficients.Rows}x{distortionCoefficients.Cols}");
            //Console.WriteLine($"Camera Matrix Content:\n{cameraMatrix.Dump()}");
            //Console.WriteLine($"Distortion Coefficients Content:\n{distortionCoefficients.Dump()}");
        }

        public static void LoadCalibrationManual(string filePath, out Mat cameraMatrix, out Mat distortionCoefficients)
        {
            var doc = XDocument.Load(filePath);

            // cameraMatrix 읽기
            var cameraMatrixNode = doc.Root.Element("cameraMatrix").Element("data").Value.Trim();
            cameraMatrix = new Mat(3, 3, MatType.CV_64FC1);
            var cameraMatrixValues = cameraMatrixNode.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            for (int i = 0; i < cameraMatrixValues.Length; i++)
            {
                cameraMatrix.Set(i / 3, i % 3, double.Parse(cameraMatrixValues[i]));
            }

            // distortionCoefficients 읽기
            var distortionCoefficientsNode = doc.Root.Element("distortionCoefficients").Element("data").Value.Trim();
            distortionCoefficients = new Mat(1, 5, MatType.CV_64FC1);
            var distortionCoefficientsValues = distortionCoefficientsNode.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            for (int i = 0; i < distortionCoefficientsValues.Length; i++)
            {
                distortionCoefficients.Set(0, i, double.Parse(distortionCoefficientsValues[i]));
            }

            //Console.WriteLine($"Loaded Camera Matrix: {cameraMatrix.Rows}x{cameraMatrix.Cols}");
            //Console.WriteLine($"Loaded Distortion Coefficients: {distortionCoefficients.Rows}x{distortionCoefficients.Cols}");
            //Console.WriteLine($"Camera Matrix Content:\n{cameraMatrix.Dump()}");
            //Console.WriteLine($"Distortion Coefficients Content:\n{distortionCoefficients.Dump()}");
        }
    }
}
