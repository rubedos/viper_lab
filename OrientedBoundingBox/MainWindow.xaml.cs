using Ros.Net.utilities;
using Rubedos.RosToolsApplicationBase.Pointcloud;
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
using HelixToolkit.Wpf.SharpDX;
using SharpDX;

namespace GroundTracker
{
  /// <summary>
  /// This sample requires VIPER to be hanged vertically above flat scene of observation. Objects placed inside the scene are 
  /// surrounded by bounding box in realtime.
  /// </summary>
  public partial class MainWindow : Window
  {
    #region privates
    private static readonly log4net.ILog log = log4net.LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

    /// <summary>
    /// View model of PointClout
    /// </summary>
    private PointcloudViewModel pointcloudViewModel;

    /// <summary>
    /// Distance from camera to the ground
    /// </summary>
    float groundZ = 1.88f;

    /// <summary>
    /// Width of the observed area on the ground
    /// </summary>
    float groundW = 1.5f;

    /// <summary>
    /// Depth of the observed area on the ground
    /// </summary>
    float groundD = 0.8f;

    /// <summary>
    /// Only take into account points that are above ground specified by this amount
    /// </summary>
    float groundElevation = 0.08f;

    /// <summary>
    /// Bounding box 3D models
    /// </summary>
    GroupModel3D boundingBoxGroup;

    /// <summary>
    /// For 3D label displaying
    /// </summary>
    BillboardText3D labels;

    /// <summary>
    /// The material that bounding box is assigned
    /// </summary>
    Material BoundingBoxMaterial;

    /// <summary>
    /// RGBD points buffer
    /// </summary>
    float[] points = null;

    /// <summary>
    /// Flag specifies if box or convex cull should be bounding shape
    /// </summary>
    bool showConvexHullBB = false;

    #endregion

    #region Constructor / Initialization
    /// <summary>
    /// Constructor
    /// </summary>
    public MainWindow()
    {
      log.Info("Logging on");
      Closing += MainWindow_Closing;

      InitializeComponent();
      InitializeBusinessLogic();
    }

    /// <summary>
    /// Initialization
    /// </summary>
    public void InitializeBusinessLogic()
    {
      log.Debug(Properties.Settings.Default.CameraInfoTopic); // NOTE: This line triggers setting loading
      ConfigurationHelper.RegisterSettings(Properties.Settings.Default);
      pointcloudViewModel = new PointcloudViewModel(new SharpDX.Size2(0, 0));
      PointcloudView.ViewModel = pointcloudViewModel;
      PointcloudView.InitializeScene();
      DataContext = pointcloudViewModel;

      // Moving Camera vertically up and turning it to look down
      PointcloudView.ViewModel.SetCvmPosition(new System.Windows.Media.Media3D.Vector3D(0, 0, groundZ),
        new System.Windows.Media.Media3D.Vector3D(180, 0, 90));

      var planeColor = System.Windows.Media.Color.FromArgb(20, 50, 255, 50).ToColor4();
      // Only points from this rectangular area are included and outliers are ignored
      var scenePlane = PointcloudViewModel.CreatePlane(planeColor, groundD, groundW, new SharpDX.Vector3(0, 0, 1f));
      PointcloudView.SceneRoot.Children.Add(scenePlane);

      // For 3D texts
      BillboardTextModel3D text = new BillboardTextModel3D();
      PointcloudView.SceneRoot.Children.Add(text);
      labels = new BillboardText3D();
      text.Geometry = labels;

      boundingBoxGroup = new GroupModel3D();
      PointcloudView.SceneRoot.Children.Add(boundingBoxGroup);

      var boxColor = System.Windows.Media.Color.FromArgb(30, 50, 0, 255).ToColor4();
      BoundingBoxMaterial = new PhongMaterial()
      {
        AmbientColor = boxColor,
        DiffuseColor = boxColor,
        SpecularColor = boxColor,
        SpecularShininess = 100f,
      };


      rosControlBase.RosConnected += RosControlBase_RosConnected;
      rosControlBase.RosDisconnected += RosControlBase_RosDisconnected;
      rosControlBase.CvmDeviceInfoChaged += RosControlBase_CvmDeviceInfoChaged;
    }

    #endregion

    #region Methods

    /// <summary>
    /// Releases resources
    /// </summary>
    public void Dispose()
    {
      rosControlBase.RosConnected -= RosControlBase_RosConnected;
      rosControlBase.RosDisconnected -= RosControlBase_RosDisconnected;
      rosControlBase.CvmDeviceInfoChaged -= RosControlBase_CvmDeviceInfoChaged;
      pointcloudViewModel.Dispose();
      rosControlBase.Device.Dispose();
    }

    #endregion

    #region Event handlers

    /// <summary>
    /// Event handler is called when CVM device info has changed
    /// </summary>
    /// <param name="sender">Sender</param>
    /// <param name="e">Arguments</param>
    private void RosControlBase_CvmDeviceInfoChaged(object sender, EventArgs e)
    {
      double f = rosControlBase.Device.DeviceInfo.FocalPoint;
      double B = rosControlBase.Device.DeviceInfo.Baseline;

      if (pointcloudViewModel.ImagingPipeline != null)
      {
        pointcloudViewModel.ImagingPipeline.SetCameraInfo(B, f, rosControlBase.Device.DeviceInfo.PrincipalPoint);
      }
      PointcloudView.AddFov(rosControlBase.Device.DeviceInfo.FovV, rosControlBase.Device.DeviceInfo.FovH, 1f, 10f, rosControlBase.Device.DeviceInfo.Baseline);
    }

    /// <summary>
    /// Event handler is called when successfully connected to ROS master server
    /// </summary>
    /// <param name="sender">Sender</param>
    /// <param name="e">Arguments</param>
    private void RosControlBase_RosConnected(object sender, EventArgs e)
    {
      try
      {
        // NOTE: GPU filters can be initialized only when View3D of HelixToolkit has been launched.
        PointcloudView.InitializePipeline(Properties.Settings.Default.ForceCpuFiltering, 1);
        if (rosControlBase.Device.DeviceInfo != null)
        {
          pointcloudViewModel.ImagingPipeline.SetCameraInfo(rosControlBase.Device.DeviceInfo.Baseline,
            rosControlBase.Device.DeviceInfo.FocalPoint, rosControlBase.Device.DeviceInfo.PrincipalPoint);
        }
        pointcloudViewModel.ImagingPipeline.ImageDataProcessed += ImagingPipeline_ImageDataProcessed;
      }
      catch (Exception ex)
      {
        MessageBox.Show($"Fatal error initializing pipeline: {ex.GetBaseException().Message}");
        throw;
      }

    }

    /// <summary>
    /// Convenience method - extract point from RGBD buffer
    /// </summary>
    /// <param name="buffer">the point cloud</param>
    /// <param name="x">x</param>
    /// <param name="y">y</param>
    /// <param name="w">width</param>
    /// <param name="h">height</param>
    /// <returns></returns>
    Vector3 GetPoint(float[] buffer, int x, int y, int w, int h)
    {
      float X = buffer[(y * w + x) * 4 + 1];
      float Y = buffer[(y * w + x) * 4 + 2];
      float Z = buffer[(y * w + x) * 4 + 3];
      return new Vector3(X, Y, Z);
    }

    /// <summary>
    /// Process point cloud as soon as it arrives
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    private void ImagingPipeline_ImageDataProcessed(object sender, Rubedos.PointcloudProcessing.ImagingPipelineProcessedEventArgs e)
    {
      var rgbd = pointcloudViewModel.ImagingPipeline.RgbdOut;
      if (points == null)
      {
        points = new float[rgbd.Cols * rgbd.Rows * rgbd.NumberOfChannels];
      }
      int w = rgbd.Cols, h = rgbd.Rows;
      // hint: working with data buffer is much faster than accessing individual points in Cv.Mat
      System.Runtime.InteropServices.Marshal.Copy(rgbd.DataPointer, points, 0, points.Length);

      ConvexHullBoundingBox(w, h, points);
    }

    /// <summary>
    /// Finds convex hull of the pointcloud flattened on XY plane. Because points are organized in a grid of w x h size, convex hull
    /// can be determined from right or left sides (thus - clockwise or counterclokwise). Final result shall have only half the convex hull
    /// so to get full polygon this function must be called twice and then merge them.
    /// NOTE: using Graham Scan algorithm.
    /// https://stackoverflow.com/questions/34479435/fit-rectangle-around-points
    /// </summary>
    /// <param name="w"></param>
    /// <param name="h"></param>
    /// <param name="pointBuffer">pointcloud</param>
    /// <param name="cwSearch">is it clockwise search?</param>
    /// <returns></returns>
    private List<Vector2> FindConvexHull(int w, int h, byte[] pointBuffer, bool cwSearch)
    {
      List<Vector2> convexHull = new List<Vector2>();
      int fromX = 0, toX = w, step = 1;
      if (cwSearch)
      {
        fromX = w - 1;
        toX = -1;
        step = -1;
      }
      for (int y = 0; y < h; y += 1)
        for (int x = fromX; x != toX; x += step)
        {
          if (pointBuffer[x + y * w] == 0)
            continue;
          Vector2 current = new Vector2(x, y);
          while (convexHull.Count > 1)
          {
            var p1 = convexHull[convexHull.Count - 2];
            var p2 = convexHull[convexHull.Count - 1];
            var v1 = p2 - p1;
            var v2 = current - p2;
            if (Vector3.Cross(new Vector3(v1.X, v1.Y, 0), new Vector3(v2.X, v2.Y, 0)).Z < 0) // left turn
            {
              if (!cwSearch)
                break;
            }
            else if (cwSearch)
              break;
            convexHull.RemoveAt(convexHull.Count - 1);
          }
          convexHull.Add(current);
          break;
        }

      return convexHull;
    }
    bool CheckNAN(Vector2 v)
    {
      if (double.IsNaN(v.X) || double.IsNaN(v.Y))
        return true;
      return false;
    }

    /// <summary>
    /// Finds minimal bounding box around convex hull
    /// </summary>
    /// <param name="convHull"></param>
    private List<Vector2> ConvexHullToBoundingBox(List<Vector2> convHull)
    {
      // Theory is that at least one edge of convex hull belongs to the bounding box. 
      // So iterate through edges and find max distances along perpendicular axes of this edge. Smallest BB wins
      float smin = float.MaxValue;
      List<Vector2> boundingBox = new List<Vector2>();
      for (int i = 0; i < convHull.Count -1; i++)
      {
        Vector2 edge = convHull[i + 1] - convHull[i];
        // u - colinear to edge, v - perpendicular.
        var u = new Vector(edge.X, edge.Y);
        if (u.Length < 1E-6)
          continue;
        u.Normalize();
        var m = new System.Windows.Media.Matrix();
        m.M11 = u.X;
        m.M12 = -u.Y;
        m.M21 = u.Y;
        m.M22 = u.X;
        Vector origin = new Vector(-convHull[i].X, -convHull[i].Y);
        Vector2 max = new Vector2(),
          min = new Vector2();
        for (int j = 0; j < convHull.Count; j++)
        {
          if (i == j) continue;
          var vec = convHull[j];
          var uvec = m.Transform(new Vector(vec.X, vec.Y) + origin);
          if (uvec.X < min.X) min.X = (float)uvec.X;
          if (uvec.Y < min.Y) min.Y = (float)uvec.Y;
          if (uvec.X > max.X) max.X = (float)uvec.X;
          if (uvec.Y > max.Y) max.Y = (float)uvec.Y;
        }
        float s = (max.X - min.X) * (max.Y - min.Y);
        if (s < smin)
        {
          smin = s;
          boundingBox = new List<Vector2>();
          // rectangle corner points
          m.Invert();
          var p1 = (m.Transform(new Vector(min.X, min.Y)) - origin);
          var p2 = (m.Transform(new Vector(min.X, max.Y)) - origin);
          var p3 = (m.Transform(new Vector(max.X, max.Y)) - origin);
          var p4 = (m.Transform(new Vector(max.X, min.Y)) - origin);
          boundingBox.Add(new Vector2((float)p1.X, (float)p1.Y));
          boundingBox.Add(new Vector2((float)p2.X, (float)p2.Y));
          boundingBox.Add(new Vector2((float)p3.X, (float)p3.Y));
          boundingBox.Add(new Vector2((float)p4.X, (float)p4.Y));
          boundingBox.Add(boundingBox.First());
        }
      }
      return boundingBox;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="w"></param>
    /// <param name="h"></param>
    /// <param name="pointBuffer"></param>
    private void ConvexHullBoundingBox(int w, int h, float[] pointBuffer)
    {
      // Flatten pointcloud first
      float cellSizeW = groundW / w;
      float cellSizeH = groundD / h;
      byte[] flatView = new byte[w * h];
      float minZ = groundZ - groundElevation;
      int pointCount = 0;
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
        {
          var cp = GetPoint(points, x, y, w, h);
          if (!IsInScene(cp))
            continue;
          pointCount++;
          cp = cp + new Vector3(groundW / 2, groundD / 2, 0); // move to center
          if (cp.Z < minZ)
            minZ = cp.Z;
          int ix = (int)(cp.X / cellSizeW + 0.5);
          int iy = (int)(cp.Y / cellSizeH + 0.5);
          if (ix < 0 || iy < 0 || ix >= w || iy >= h)
            continue;
          flatView[ix + iy * w] = 255;
        }
      if (pointCount == 0)
        return;
      var convexHullRight = FindConvexHull(w, h, flatView, true);
      var convexHullLeft = FindConvexHull(w, h, flatView, false);

      // Loop closure
      foreach (var p in convexHullLeft)
        convexHullRight.Insert(0, p);
      convexHullRight.Insert(0, convexHullRight.Last());
  
      if (!showConvexHullBB) convexHullRight = ConvexHullToBoundingBox(convexHullRight);

      // Synchronizing with 3D rendering thread
      pointcloudViewModel.Context.Send((o) =>
      {
        boundingBoxGroup.Children.Clear();
        MeshBuilder mb = new MeshBuilder();
        mb.AddPolygon(convexHullRight, new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, groundZ));

        MeshGeometryModel3D mmodel = new MeshGeometryModel3D();
        mmodel.Material = BoundingBoxMaterial;
        //boundingBoxGroup.Children.Add(mmodel);

        LineGeometryModel3D linem = new LineGeometryModel3D();
        var lb = new LineBuilder();
        linem.Thickness = 1;
        linem.Color = Colors.Yellow;
        Vector3 boxH = new Vector3(0, 0, groundZ - minZ);
        Vector3? last = null;
        var center = new Vector3();
        int pointNum = 0;
        float boxW = 0f, boxD = 0f;
        foreach (var p in convexHullRight)
        {
          Vector3 point = new Vector3(p.Y * cellSizeH - groundD / 2, p.X * cellSizeW - groundW / 2, 0);
          center += point;
          if (last != null)
          {
            lb.AddLine(last.Value, point);
            lb.AddLine(last.Value + boxH, point + boxH);
            lb.AddLine(last.Value, last.Value + boxH);
            lb.AddLine(point, point + boxH);
            mb.AddQuad(last.Value, point, point + boxH, last.Value + boxH);
          }
          if (pointNum == 1)
          {
            boxW = (point - last).Value.Length();
          }
          if (pointNum == 2)
          {
            boxD = (point - last).Value.Length();
          }
          last = point;
          pointNum++;
        }
        mmodel.Geometry = mb.ToMeshGeometry3D();
        linem.Geometry = lb.ToLineGeometry3D();
        boundingBoxGroup.Children.Add(linem);
        center = center / convexHullRight.Count;
        float scale = 1.0f;
        labels.TextInfo.Clear();
        labels.TextInfo.Add(new TextInfo()
        {
          Text = String.Format("H = {0:0.0} m", boxH.Z),
          Origin = new Vector3(0, 0, boxH.Z + 0.1f) + center,
          Foreground = Colors.Black.ToColor4(),
          Scale = scale
        });
        if (!showConvexHullBB && convexHullRight.Count > 2)
        {
          labels.TextInfo.Add(new TextInfo()
          {
            Text = String.Format("W = {0:0.0} m", boxW),
            Origin = new Vector3(0, boxD / 2 + 0.1f, boxH.Z + 0.1f) + center,
            Foreground = Colors.Black.ToColor4(),
            Scale = scale
          });

          labels.TextInfo.Add(new TextInfo()
          {
            Text = String.Format("D = {0:0.0} m", boxD),
            Origin = new Vector3(boxW / 2 + 0.1f, 0f, boxH.Z + 0.1f) + center,
            Foreground = Colors.Black.ToColor4(),
            Scale = scale
          });
        }

      }, null);
    }

    /// <summary>
    /// Checks if point is within scene boundaries
    /// </summary>
    /// <param name="p"></param>
    /// <returns></returns>
    private bool IsInScene(Vector3 p)
    {
      if (p.Z > groundZ - groundElevation || p.Z < 1.0f ||
        p.X > groundW / 2 || p.X < -groundW / 2 ||
        p.Y > groundD / 2 || p.Y < -groundD / 2)
        return false;
      return true;
    }

    /// <summary>
    /// Event handler is called when disconnected from ROS master server
    /// </summary>
    /// <param name="sender">Sender</param>
    /// <param name="e">Arguments</param>
    private void RosControlBase_RosDisconnected(object sender, EventArgs e)
    {
      MessageBox.Show("Disconnected.");
    }

    /// <summary>
    /// Event handler is called when Application started exiting
    /// </summary>
    /// <param name="sender">Sender</param>
    /// <param name="e">Arguments</param>
    private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
    {
      Closing -= MainWindow_Closing;
      Dispose();
    }

    #endregion

    private void convexHullCB_Checked(object sender, RoutedEventArgs e)
    {
      showConvexHullBB = convexHullCB.IsChecked.Value;
    }
  }
}
