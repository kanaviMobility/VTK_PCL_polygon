//using PCL and VTK
//generate Rectangle and ARC on XY_Plane

//Generate Plane
vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients &coeffi, double x, double y, double z, PointXYZ point1, PointXYZ point2);
vtkSmartPointer<vtkPolyData> createPlane_XY(double x, double y, double z, PointXYZ point1, PointXYZ point2);
//generate Arc
vtkSmartPointer<vtkPolyData> createArc(double FoV, double Start_Angle, double Length, PointXYZ Center, int resolution, bool Negative);

vtkSmartPointer<vtkPolyData> Processor_Grid::createPlane(const ModelCoefficients &coeffi, double x, double y, double z, PointXYZ point1, PointXYZ point2)
{
    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();

    double norm_sqr = 1.0 / static_cast<double>((coeffi.values[0] * coeffi.values[0] + coeffi.values[1] * coeffi.values[1] + coeffi.values[2] * coeffi.values[2]));

    plane->SetNormal(static_cast<double>(coeffi.values[0]), static_cast<double>(coeffi.values[1]), static_cast<double>(coeffi.values[2]));
    double t = x * static_cast<double>(coeffi.values[0]) + y * static_cast<double>(coeffi.values[1]) + z * static_cast<double>(coeffi.values[2]) + static_cast<double>(coeffi.values[3]);

    x -= static_cast<double>(coeffi.values[0]) * t * norm_sqr;
    y -= static_cast<double>(coeffi.values[1]) * t * norm_sqr;
    z -= static_cast<double>(coeffi.values[2]) * t * norm_sqr;

    plane->SetOrigin(x,y,z);
    plane->SetPoint1(static_cast<double>(point1.x), static_cast<double>(point1.y), static_cast<double>(point1.z));
    plane->SetPoint2(static_cast<double>(point2.x), static_cast<double>(point2.y), static_cast<double>(point2.z));

    plane->Update();

    return (plane->GetOutput());
}

vtkSmartPointer<vtkPolyData> Processor_Grid::createPlane_XY(double x, double y, double z, PointXYZ point1, PointXYZ point2)
{
    pcl::ModelCoefficients plane_coeff;
    plane_coeff.values.resize(4);   //Plane XY
    plane_coeff.values[0] = 0;
    plane_coeff.values[1] = 0;
    plane_coeff.values[2] = 1;
    plane_coeff.values[3] = 0;

    return createPlane(plane_coeff, x,y,z, point1, point2);
}

vtkSmartPointer<vtkPolyData> Processor_Grid::createArc(double FoV, double Start_Angle, double Length, PointXYZ Center, int resolution, bool Negative)
{
    vtkSmartPointer<vtkArcSource> Arc = vtkSmartPointer<vtkArcSource>::New();

    //set Point 01
    double sin_V = sin(pcl::deg2rad(Start_Angle));
    double cos_V = cos(pcl::deg2rad(Start_Angle));
    double p1_x = Length * sin_V;
    double p1_y = Length * cos_V;

    //set Point 02
    sin_V = sin(pcl::deg2rad(Start_Angle + FoV));
    cos_V = cos(pcl::deg2rad(Start_Angle + FoV));
    double p2_x = Length * sin_V;
    double p2_y = Length * cos_V;

    Arc->SetPoint1(p1_x,p1_y,0);      //set position of the first end point.
    Arc->SetPoint2(p2_x,p2_y,0);     //set position of the other end point.
    Arc->SetCenter(Center.x, Center.y, Center.z);      //set position of the center of the circl that defines the arc.
    Arc->SetResolution(resolution);     //define the number of segments of the polyline that draws the arc -- set to 1, the arc is straight line
    Arc->SetNegative(Negative);

    Arc->SetUseNormalAndAngle(false);
    Arc->SetPolarVector(1,0,0); //set polar vector -- only used when UseNormalAndAngle is ON.
    Arc->SetNormal(0,0,1);      //set normal Vector -- only used when UseNormalAndAngle is ON.
    Arc->SetAngle(145);          //set Arc length -- only used when UseNormalAndAngle is ON.

    Arc->Update();


    return (Arc->GetOutput());
}
