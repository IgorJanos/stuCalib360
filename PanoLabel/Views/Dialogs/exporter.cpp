#include "pch.h"
#include <random>
#include <iostream>


#include <QOpenGLExtraFunctions>

namespace Exporter {



cv::Mat _rx(double a)
{
    cv::Mat R_x = (cv::Mat_<double>(4,4) <<
        1, 0,      0,        0,
        0, cos(a), -sin(a),  0,
        0, sin(a), cos(a),   0,
        0, 0,      0,        1
    );
    return R_x;
}

cv::Mat _ry(double a)
{
    cv::Mat R_y = (cv::Mat_<double>(4,4) <<
        cos(a),    0, sin(a), 0,
        0,         1, 0,      0,
        -sin(a),   0, cos(a), 0,
        0, 0, 0,   1
    );
    return R_y;
}

cv::Mat _rz(double a)
{
    cv::Mat R_z = (cv::Mat_<double>(4,4) <<
       cos(a), -sin(a), 0, 0,
       sin(a), cos(a),  0, 0,
       0,          0,   1, 0,
       0,          0,   0, 1
    );
    return R_z;
}





Orientation::Orientation() :
    pan(0), tilt(0), roll(0)
{
}

Orientation::Orientation(double p, double t, double r) :
    pan(p),
    tilt(t),
    roll(r)
{
}

Orientation::Orientation(cv::Vec3d o) :
    pan(o[0]),
    tilt(o[1]),
    roll(o[2])
{
}

Orientation::Orientation(const Orientation &o):
    pan(o.pan), tilt(o.tilt), roll(o.roll)
{
}

Orientation &Orientation::operator=(const Orientation &o)
{
    pan = o.pan;
    tilt = o.tilt;
    roll = o.roll;
    return *this;
}


cv::Mat Orientation::rotation_zxz()
{
    cv::Mat R = (cv::Mat_<double>(4,4) <<
        -sin(pan)*sin(roll)*cos(tilt) +  cos(pan)*cos(roll),
         sin(pan)*cos(roll) + sin(roll)*cos(pan)*cos(tilt),
         sin(roll)*sin(tilt),
         0,

        -sin(pan)*cos(roll)*cos(tilt) -  sin(roll)*cos(pan),
        -sin(pan)*sin(roll) + cos(pan)*cos(roll)*cos(tilt),
         sin(tilt)*cos(roll),
         0,

         sin(pan)*sin(tilt),
        -sin(tilt)*cos(pan),
         cos(tilt),
         0,

        0, 0, 0, 1
    );
    return R;
}

cv::Mat Orientation::rotation_xyz()
{
    return _rotate_rad(cv::Vec3d(
            tilt,    // X-axis
            pan,     // Y-axis
            roll     // Z-axis
        ));
}


Orientation Orientation::zxz_from_rotation(cv::Mat R)
{
    double p = 0;
    double t = 0;
    double r = 0;

    double st = sqrt(R.at<double>(0,2) * R.at<double>(0,2) +  R.at<double>(1,2) * R.at<double>(1,2) );
    bool singular = st < 1e-6; // If

    if (!singular)
    {
        p = atan2(R.at<double>(2,0) , -R.at<double>(2,1));
        r = atan2(R.at<double>(0,2), R.at<double>(1,2));                
        t = atan2(st, R.at<double>(2,2));
    }
    else
    {
        //x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        //y = atan2(-R.at<double>(2,0), sy);
        //tilt = 0;
        std::cout << "ASDASD !!!!" << std::endl;
    }
    return Orientation(p, t, r);
}



//-----------------------------------------------------------------------------
//
//  Exporting API
//
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
//  Matrix Helpers
//-----------------------------------------------------------------------------

cv::Mat _toSoccerNet()
{
    cv::Mat     K = (cv::Mat_<double>(3,3) <<
        1, 0,  0,
        0, 0, -1,
        0, -1, 0
    );
    return K;
}

cv::Mat _camera(double width, double height, double fov_v_rad)
{
    double   w = width;
    double   h = height;
    double   f = h * fov_to_f(fov_v_rad);

    cv::Mat     K = (cv::Mat_<double>(3,3) <<
        f, 0, w/2.0,
        0, f, h/2.0,
        0, 0, 1
    );
    return K;
}


cv::Mat _toGL()
{
    cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);
    cvToGl.at<double>(0, 0) = 1.0f;
    cvToGl.at<double>(1, 1) = -1.0f; // Invert the y axis
    cvToGl.at<double>(2, 2) = -1.0f; // invert the z axis
    cvToGl.at<double>(3, 3) = 1.0f;
    return cvToGl;
}


double fov_to_f(double fov)
{
    return 1.0 / (2.0 * tan(fov/2.0));
}

double f_to_fov(double f, double x)
{
    return 2.0 * atan2(x, (2.0 * f));
}

cv::Mat _extract_rotation_from_4x4(cv::Mat m)
{
    cv::Mat     r = (cv::Mat_<double>(3, 3) <<
        m.at<double>(0,0),    m.at<double>(0,1), m.at<double>(0,2),
        m.at<double>(1,0),    m.at<double>(1,1), m.at<double>(1,2),
        m.at<double>(2,0),    m.at<double>(2,1), m.at<double>(2,2)
        );
    return r;
}

cv::Mat _mv_from_rv_tv(cv::Mat rvec, cv::Mat tvec)
{
    // Compute the rotation matrix
    cv::Mat     R;
    try {
        cv::Rodrigues(rvec, R);
    }
    catch (...) {
    }

    cv::Mat     m = (cv::Mat_<double>(4, 4) <<
         R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2),
         0, 0, 0, 1
        );
    return m;
}

cv::Mat _identity()
{
    cv::Mat T = (cv::Mat_<double>(4,4) <<
                     1,  0,  0,  0,
                 0,  1,  0,  0,
                 0,  0,  1,  0,
                 0,  0,  0,  1
                 );
    return T;
}

cv::Mat _fromQ(QMatrix4x4 m)
{
    float *f = (float*)m.data();
    cv::Mat     result = (cv::Mat_<double>(4,4) <<
                          f[0], f[1], f[2], f[3],
                      f[4], f[5], f[6], f[7],
                      f[8], f[9], f[10], f[11],
                      f[12], f[13], f[14], f[15]
                      );
    return result.t();
}

QMatrix4x4 _toQ(cv::Mat m)
{
    m.convertTo(m, CV_32F);
    return QMatrix4x4((const float*)m.data);
}


cv::Mat _translate(cv::Vec3d v)
{
    cv::Mat T = (cv::Mat_<double>(4,4) <<
                 1,  0,  0, (double)v[0],
                 0,  1,  0, (double)v[1],
                 0,  0,  1, (double)v[2],
                 0,  0,  0,  1
                 );
    return T;
}

cv::Mat _rotate_rad(cv::Vec3d a)
{
    cv::Mat R_x = (cv::Mat_<double>(4,4) <<
                       1, 0,            0,       0,
                   0, cos(a[0]),   -sin(a[0]),   0,
                   0, sin(a[0]),    cos(a[0]),   0,
                   0, 0, 0,                      1
                   );

    cv::Mat R_y = (cv::Mat_<double>(4,4) <<
                   cos(a[1]),    0, sin(a[1]),   0,
                   0,            1, 0,           0,
                   -sin(a[1]),   0, cos(a[1]),   0,
                   0, 0, 0,                      1
                   );

    cv::Mat R_z = (cv::Mat_<double>(4,4) <<
                   cos(a[2]), -sin(a[2]),    0, 0,
                   sin(a[2]), cos(a[2]),     0, 0,
                   0,          0,            1, 0,
                   0,          0,            0, 1
                   );
    cv::Mat R = R_z * R_y * R_x;
    return R;

}

cv::Mat _rotate_deg(cv::Vec3d a)
{
    return _rotate_rad(a * M_PI / 180.0);
}

cv::Vec3d _transform(cv::Vec3d v, cv::Mat C)
{
    cv::Vec4d     v4(v[0], v[1], v[2], 1.0);
    cv::Mat p = cv::Mat(v4);
    cv::Mat r = C * p;

    return cv::Vec3d(r.at<double>(0,0), r.at<double>(1,0), r.at<double>(2,0));
}

static cv::Vec3d _normalize(cv::Vec3d v)
{
    return cv::normalize(v);
}

cv::Mat _lookat(
    cv::Vec3d eye,
    cv::Vec3d target,
    cv::Vec3d up
    )
{
    cv::Vec3d f = _normalize(target - eye);
    cv::Vec3d s = _normalize(f.cross(up));
    cv::Vec3d u = _normalize(s.cross(f));


    cv::Mat R = (cv::Mat_<double>(4,4) <<
                 s[0], u[0], f[0], 0,
                 s[1], u[1], f[1], 0,
                 s[2], u[2], f[2], 0,
                 0,    0,    0,    1
                 );

    cv::Mat T = _translate(-eye);
    cv::Mat result = R.inv()*T;
    return result;
}



// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    return cv::norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R)
{
    //assert(isRotationMatrix(R));

    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3d(x, y, z);
}




cv::Mat get_panorama_inverse_V(cv::Mat V)
{
    cv::Mat  R0 = _extract_rotation_from_4x4(_rx(90 * M_PI/180.0));
    cv::Mat  R = _extract_rotation_from_4x4(V);
    // result
    return (R*R0);
}


typedef QList<std::pair<cv::Vec2d, bool>>     PointList;


PointList project_panorama_to_pinhole(
    QSharedPointer<Data::ImageMapping> amapping,
    cv::Mat K, cv::Mat P,
    double imw, double imh
    )
{
    PointList    result;
    if (imw <= 0 || imh <= 0) return result;

    if (amapping) {

        cv::Mat     r3d, pi;
        double      x, y, z;
        double      theta, phi;

        for (int i=0; i<(int)amapping->kps.size(); i++) {
            Data::KeyPointMapping &src = amapping->kps[i];

            // Angles in panorama
            theta = (2.0*M_PI * src.x / imw) - M_PI;
            phi   =      M_PI * src.y / imh - M_PI_2;

            r3d = (cv::Mat_<double>(3,1) <<
                   sin(phi) * sin(theta),
                   cos(phi),
                   sin(phi) * cos(theta)
                   );

            pi = P * r3d;
            x = pi.at<double>(0, 0) / pi.at<double>(2, 0);
            y = pi.at<double>(1, 0) / pi.at<double>(2, 0);
            z = pi.at<double>(2, 0);

            // Into camera space
            x = x*K.at<double>(0,0) + K.at<double>(0,2);
            y = y*K.at<double>(1,1) + K.at<double>(1,2);

            // mapujeme jeho x, y
            cv::Vec2d       p(x, y);
            result.push_back(std::make_pair(p, z>0));
        }
    }

    return result;
}


int _find_homography(
    QSharedPointer<Data::ImageMapping> amapping,
    PointList imagepoints,
    QRect rcClip,

    // out
    cv::Mat &K,
    cv::Mat &R, cv::Mat &T
    )
{
    std::vector<cv::Point3f>        pts3d;
    std::vector<cv::Point2f>        pts2d;

    cv::Mat     dist = cv::Mat(4,1,CV_64FC1,cv::Scalar(0));

    for (int i=0; i<(int)amapping->kps.size(); i++) {
        auto kp = amapping->kps[i];
        auto ip = imagepoints[i];

        // Check if in viewfinder
        QPoint      p((int)ip.first[0], (int)ip.first[1]);
        if (ip.second && rcClip.contains(p)) {
            pts3d.push_back(cv::Point3f(kp.kp.x, kp.kp.y, kp.kp.z));
            pts2d.push_back(cv::Point2f(ip.first[0], ip.first[1]));
        }
    }

    if (pts2d.size() < 6) return 0;

    std::vector<std::vector<cv::Point2f>>     ip;
    std::vector<std::vector<cv::Point3f>>     op;

    ip.push_back(pts2d);
    op.push_back(pts3d);

    cv::Size    size(rcClip.width(), rcClip.height());
    cv::Mat     matD;
    cv::Mat     outK = K;

    std::vector<cv::Mat>     rv, tv;
    cv::calibrateCamera(
        op, ip, size,
        outK, matD,
        rv, tv,
        0
            | cv::CALIB_USE_INTRINSIC_GUESS
            | cv::CALIB_FIX_ASPECT_RATIO
            | cv::CALIB_ZERO_TANGENT_DIST
            | cv::CALIB_FIX_PRINCIPAL_POINT
            | cv::CALIB_FIX_FOCAL_LENGTH
            | cv::CALIB_FIX_K1
            | cv::CALIB_FIX_K2
            | cv::CALIB_FIX_K3
            | cv::CALIB_FIX_K4
            | cv::CALIB_FIX_K5
            | cv::CALIB_FIX_K6
        );

    R = rv[0];
    T = tv[0];
    K = outK;

    return pts2d.size();
}




//-----------------------------------------------------------------------------
//
//  MappingCalibration
//
//-----------------------------------------------------------------------------

MappingCalibration::MappingCalibration() :
    mean_position(0,0,0),
    B_calibration(_identity())
{
}

void MappingCalibration::calibrate(
        QSharedPointer<Data::ImageMapping> amapping,
        cv::Size aimage_size
    )
{
    /*
     *      Calibrate the mean position and orientation by looking
     *      around a bit.
     */

    this->mapping = amapping;
    this->image_size = aimage_size;

    int             num_phi = 10;               // left-right
    int             num_theta = 8;              // up-down
    cv::Size        canvas_size(4000, 1500);    // any 16:9 ratio
    double          range = M_PI*0.5;           // 180-degrees

    // Look into the center of the panorama
    Orientation     default_view(cv::Vec3d(0,90,0) * M_PI/180.0);
    cv::Mat         V_Base = default_view.rotation_zxz();


    cv::Vec3d       camera_position_acc(0.0, 0.0, 0.0);
    cv::Vec3d       orientation_acc(0.0, 0.0, 0.0);
    int             count = 0;

    for (int i=0; i<(num_theta+1); i++) {
        for (int j=0; j<(num_phi+1); j++) {

            // Compute angle
            Orientation     current_view;
            current_view.pan = -(range/2.0) + (i*range/num_theta);
            current_view.tilt = -(range/2.0) + (j*range/num_phi);
            cv::Mat         V_Current = current_view.rotation_xyz();

            // Total View transform
            cv::Mat         V = V_Current * V_Base;

            cv::Vec3d       p, rvec;
            if (get_position_orientation(
                    amapping,
                    V,
                    90*(M_PI/180.0),                // 90 degrees vertical FOV!
                    aimage_size, canvas_size,
                    10,                             // We need more points!
                    p, rvec
                    )
                ) {

                // Compute the mean
                camera_position_acc += p;
                orientation_acc += rvec;
                count += 1;
            }
        }
    }

    cv::Vec3d rvec;
    if (count > 0) {
        mean_position = camera_position_acc / (double)count;
        rvec = orientation_acc / (double)count;
    } else {
        mean_position = cv::Vec3d(0,0,0);
        rvec = cv::Vec3d(0,0,0);
    }       

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    mean_orientation = Orientation::zxz_from_rotation(R);

    // Extract original B matrix
    pos_orientation_to_mv(
        mean_position,
        mean_orientation,
        cv::Mat::eye(4, 4, CV_64F),
        B_calibration
        );

}

void MappingCalibration::set_view(Orientation o, double fov_v_rad)
{
    // Store our View matrix
    view_orientation = o;
    V = view_orientation.rotation_zxz();
    //V = view_orientation.rotation_xyz();

    // Optimize the view
    optimize_view(fov_v_rad);
}


bool MappingCalibration::optimize_view(double fov_v_rad)
{

    double fov_min = fmax(10.0f, 0.5*fov_v_rad*180/M_PI);
    double fov_max = 50;
    int fov_steps = 3;

    cv::Vec3d       camera_position_acc(0.0, 0.0, 0.0);
    cv::Vec3d       orientation_acc(0.0, 0.0, 0.0);
    double          count = 0.0;
    double          step = 2.5 * M_PI/180.0;

    float   W[10] = {
        1.5, 1.5, 1.0, 1.0, 1.0,
        0.8, 0.8, 0.6, 0.6, 0.5
    };

    cv::Mat         V_Base = this->V;

    //std::cout << "--- Started " << std::endl;
    // Scan the whole FOV range - and one step left/right
    for (int stepX=-1; stepX<2; stepX++) {

        // Compute angle
        Orientation     current_view;
        current_view.pan += (stepX * step);
        cv::Mat         V_Current = current_view.rotation_xyz();

        // Total View transform
        cv::Mat         V = V_Current * V_Base;

        for (int i=0; i<=fov_steps; i++) {
            double      fov = fov_min + (fov_max - fov_min)*((double)i / (double)fov_steps);

            cv::Vec3d   p, rvec;
            if (get_position_orientation(
                    mapping,
                    V,
                    fov * M_PI/180.0,
                    image_size,
                    cv::Size(3840, 2160), //aview_size,     // Fixed virtual resolution 16:9
                    8,
                    p, rvec
                    )
                ) {

                //std::cout << " - " << i << "   P: " << p << "    O: " << (o*180.0/M_PI) << std::endl;
                // Make it into a new matrix
                camera_position_acc += (p * W[i]);
                orientation_acc += rvec * W[i];
                count += W[i];
            }
        }
    }


    if (count > 0) {

        cv::Vec3d position = camera_position_acc / (double)count;
        cv::Vec3d rvec = orientation_acc / (double)count;

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        Orientation o = Orientation::zxz_from_rotation(R);

        // Make it into a new matrix
        pos_orientation_to_mv(
            position, o,
            cv::Mat::eye(4,4,CV_64F),
            B);

        // Store refined parameters
        base_position = position;
        base_orientation = o;
        return true;
    }
    return false;
}

void MappingCalibration::look_at(cv::Vec3d target, double fov_v_rad)
{
    // Initial - start from mean position
    cv::Mat     mv = _lookat(mean_position, target, cv::Vec3d(0,0,1));
    cv::Mat     newV = (mv * B_calibration.inv());
    Orientation o = Orientation::zxz_from_rotation(newV);
    set_view(o, fov_v_rad);

    // Iterate!
    mv = _lookat(base_position, target, cv::Vec3d(0,0,1));
    newV = (mv * B.inv());
    o = Orientation::zxz_from_rotation(newV);
    set_view(o, fov_v_rad);

    // Level roll
    Orientation absO = Orientation::zxz_from_rotation(V*B);
    view_orientation.roll -= absO.roll;
    V = view_orientation.rotation_zxz();
}


bool MappingCalibration::get_position_orientation(
    QSharedPointer<Data::ImageMapping> amapping,
    cv::Mat V, double fov_v_rad,
    cv::Size image_size, cv::Size view_size,
    int min_num_points,
    cv::Vec3d &position, cv::Vec3d &rvec
    )
{
    double      w = view_size.width;
    double      h = view_size.height;
    QRect       rc_clip(0, 0, (int)w, (int)h);

    // Get our inverse RK matrix
    cv::Mat     K = _camera(w, h, fov_v_rad);
    cv::Mat     P = get_panorama_inverse_V(V);

    // Look around
    auto points = project_panorama_to_pinhole(
            amapping, K, P,
            (double)image_size.width, (double)image_size.height
        );

    // Camera matrix with pixel focal width

    // Find model homography
    cv::Mat     hrvec, tvec;
    int         point_count = _find_homography(amapping, points, rc_clip, K, hrvec, tvec);

    if (point_count >= min_num_points) {


        // Current MV based on rotation/translation
        cv::Mat     mv = _mv_from_rv_tv(hrvec, tvec);

        // Find new base position and orientation
        cv::Mat     b_rot;
        mv_to_pos_orientation(mv, V, position, b_rot);
        cv::Rodrigues(b_rot, rvec);
        return true;

    }

    // Just empty vectors
    position = cv::Vec3d(0,0,0);
    return false;
}


void MappingCalibration::mv_to_pos_orientation(
        cv::Mat mv, cv::Mat V,
        cv::Vec3d &p, cv::Mat &b_rot
    )
{
    /*
     *      Dcompose the model view matrix into position and orientation
     *      given the difference view orientation.
     */

    cv::Mat b = V.inv() * mv;
    cv::Mat b_inv = b.inv();

    // Camera position
    p = cv::Vec3d(
        b_inv.at<double>(0, 3),
        b_inv.at<double>(1, 3),
        b_inv.at<double>(2, 3)
    );

    // Extract the rotation part of VM
    b_rot = _extract_rotation_from_4x4(b);
}


void MappingCalibration::pos_orientation_to_mv(
    cv::Vec3d p, Orientation base_orientation,
    cv::Mat V, cv::Mat &mv
    )
{
    /*
     *      Build a new model view matrix by applying the
     *      base position/orientation with view orientation
     */

    cv::Vec3d   eye = p;
    cv::Mat     T = _translate(-eye);
    cv::Mat     BR = base_orientation.rotation_zxz();

    // Translate -> BaseRotate -> ViewRotate
    mv = V * BR * T;
}


HomographyGT MappingCalibration::get_homography_gt(
        cv::Size image_size,
        double fov_v_rad
    )
{
    HomographyGT result;

    // Final MV
    cv::Mat     mv = this->V * this->B;

    // Final orientation extracted from MV
    cv::Mat     b_rot;
    mv_to_pos_orientation(
        mv, cv::Mat::eye(4, 4, CV_64F),
        result.position,
        b_rot
       );

    //result.position = mean_position;
    result.orientation = Orientation::zxz_from_rotation(b_rot);
    result.fov_v = fov_v_rad * 180.0/M_PI;

    // Convert FOV
    double f = fov_to_f(fov_v_rad);
    double a = (double)image_size.width / (double)image_size.height;
    result.fov_h = 2.0 * atan2(a, 2.0*f) * 180.0/M_PI;

    return result;
}


PointList MappingCalibration::project_for_view(cv::Mat K)
{
    cv::Mat B = _extract_rotation_from_4x4(_rx(90 * M_PI/180.0));
    cv::Mat R = _extract_rotation_from_4x4(V);
    cv::Mat P = R*B;

    PointList result = project_panorama_to_pinhole(
            mapping,
            K, P,
            (double)image_size.width,
            (double)image_size.height
        );

    return result;
}



//-----------------------------------------------------------------------------
//  Random helpers
//-----------------------------------------------------------------------------

std::default_random_engine		generator(
        std::chrono::system_clock::now().time_since_epoch().count()
        );

float uniform(std::pair<float,float> args)
{
    std::uniform_real_distribution<float> d(args.first, args.second);
    return d(generator);
}

float normal(std::pair<float,float> args)
{
    std::normal_distribution<float> d(args.first, args.second);
    return d(generator);
}


//-----------------------------------------------------------------------------
//
//  DatasetImageSource
//
//-----------------------------------------------------------------------------

DatasetImageSource::DatasetImageSource(
        DatasetModel *amodel,
        CheckedImagesModel *achmodel,
        PlayfieldCollectionModel *pfcm
        ) :
    index(-1)
{
    if (amodel && achmodel) {

        path = amodel->Path();

        // Najdeme si model
        if (pfcm) {
            pfModel = pfcm->getPlayfieldModel(amodel->Dataset()->modelFilename);
        }

        // Skopirujeme images
        for (int i=0; i<amodel->rowCount(); i++) {

            if (achmodel->data(achmodel->index(i,0), Qt::CheckStateRole) == Qt::Checked) {
                images.push_back( amodel->getFullFilename(amodel->index(i, 0)) );
            }
        }
    }
}


QSharedPointer<Image> DatasetImageSource::current()
{
    QMutexLocker    l(&lock);

    if (index >= images.size()) return nullptr;

    QSharedPointer<Image>   result = QSharedPointer<Image>(new Image());

    // loadneme subor
    result->filename = images[index];
    result->image.load(result->filename);
    result->imageSize = cv::Size(result->image.width(), result->image.height());
    result->mappingCalibration = QSharedPointer<MappingCalibration>(
        new MappingCalibration()
        );

    // skusime loadnut este aj mapping file
    QString mf = mappingFile(images[index]);
    result->mapping = QSharedPointer<Data::ImageMapping>(
                Data::ImageMapping::loadFromFile(mf, pfModel.data())
                );

    // perform initial calibration
    result->mappingCalibration->calibrate(result->mapping, result->imageSize);

    return result;
}

void DatasetImageSource::reset()
{
    QMutexLocker    l(&lock);

    index = 0;

    // skusame sa posuvat dalej
    while (index < images.size()) {

        // skusame, ci eistuje mapping file pre tento subor
        QString     fn = images[index];
        QFileInfo   fimf(mappingFile(fn));
        if (fimf.exists()) {
            // Existuje, sme spokojni ...
            return ;
        }

        index += 1;
    }
}

void DatasetImageSource::next()
{
    QMutexLocker    l(&lock);

    if (index < images.size()) index += 1;

    // skusame sa posuvat dalej
    while (index < images.size()) {

        // skusame, ci eistuje mapping file pre tento subor
        QString     fn = images[index];
        QFileInfo   fimf(mappingFile(fn));
        if (fimf.exists()) {

            // Existuje, sme spokojni ...
            return ;
        }

        // inak ideme dalej ...
        index += 1;
    }
}

bool DatasetImageSource::hasCurrent()
{
    QMutexLocker    l(&lock);

    return (index >= 0 && index < images.size());
}


QString DatasetImageSource::mappingFile(QString image) const
{
    QFileInfo       fi(image);
    QString         mappingFile = fi.baseName() + ".json";
    return path + "/mapping/" + mappingFile;
}


//-----------------------------------------------------------------------------
//
//  CycleCounter
//
//-----------------------------------------------------------------------------

CycleCounter::CycleCounter(
        QSharedPointer<PipelineSource<Image>> asrc, int acount
        ) :
    source(asrc),
    count(acount),
    index(-1)
{
}

QSharedPointer<Image> CycleCounter::current()
{
    QMutexLocker    l(&lock);

    if (index >= count) return nullptr;

    return source->current();
}

void CycleCounter::reset()
{
    QMutexLocker    l(&lock);

    source->reset();
    index = 0;

    // ak sme na konci, reset
    while (index < count) {
        if (!source->hasCurrent()) {
            source->reset();
            index += 1;
        } else {
            break;
        }
    }
}

void CycleCounter::next()
{
    QMutexLocker    l(&lock);

    if (index >= count) return ;

    // delegujeme
    source->next();

    // ak sme na konci, reset
    while (index < count) {
        if (!source->hasCurrent()) {
            source->reset();
            index += 1;
        } else {
            break;
        }
    }
}

bool CycleCounter::hasCurrent()
{
    QMutexLocker    l(&lock);

    if (index >= count) return false;

    // delegujeme
    return source->hasCurrent();
}

//-----------------------------------------------------------------------------
//
//  Repeater
//
//-----------------------------------------------------------------------------

Repeater::Repeater(
        QSharedPointer<PipelineSource<Image>> asrc, int acount
        ) :
    source(asrc),
    count(acount),
    index(0)
{


}

QSharedPointer<Image> Repeater::current()
{
    QMutexLocker    l(&lock);

    if (!cache && index == 0) {
        cache = source->current();
    }

    return cache;
}

void Repeater::reset()
{
    QMutexLocker    l(&lock);

    source->reset();
    index = 0;
    cache = nullptr;
}

void Repeater::next()
{
    QMutexLocker    l(&lock);

    index ++;
    if (index >= count) {
        cache = nullptr;
        index = 0;
        source->next();
    }
}

bool Repeater::hasCurrent()
{
    QMutexLocker    l(&lock);

    if (index > 0 && index < count) {
        return (cache ? true : false);
    }

    return source->hasCurrent();
}


//-----------------------------------------------------------------------------
//
//  CalibrationCameraPose
//
//-----------------------------------------------------------------------------

CalibrationCameraPose::CalibrationCameraPose() :
    x(0.0f), y(0.0f), z(0.0f),
    pan(0.0f), tilt(0.0f), roll(0.0f),
    fov_v(0.0f),
    view_orientation(0,0,0)
{
}

CalibrationCameraPose::CalibrationCameraPose(const CalibrationCameraPose &v) :
    x(v.x),
    y(v.y),
    z(v.z),
    pan(v.pan),
    tilt(v.tilt),
    roll(v.roll),
    fov_v(v.fov_v),
    view_orientation(v.view_orientation),
    mv(v.mv)
{
}

CalibrationCameraPose &CalibrationCameraPose::operator =(CalibrationCameraPose v)
{
    x = v.x;
    y = v.y;
    z = v.z;
    pan = v.pan;
    tilt = v.tilt;
    roll = v.roll;
    fov_v = v.fov_v;
    mv = v.mv;
    view_orientation = v.view_orientation;
    return *this;
}


void computeCameraPose(
        QSharedPointer<Image> image,
        CalibrationSample &sample,
        CalibrationCameraPose &result
    )
{
    auto m = image->mapping;
    auto mc = image->mappingCalibration;
    if (m && mc) {

        mc->look_at(sample.camera_target, sample.fov * M_PI/180.0);

        // Apply additional roll
        Orientation     view = mc->view_orientation;
        view.roll += (sample.roll * M_PI/180.0);
        mc->set_view(view, sample.fov * M_PI/180.0);

        // Final MV
        result.mv = mc->V * mc->B;

        cv::Vec3d       p;
        Orientation     o;
        cv::Mat         V = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat         R;
        mc->mv_to_pos_orientation(result.mv, V, p, R);
        o = Orientation::zxz_from_rotation(R);

        // Final orientation extracted from MV
        result.x = p[0];
        result.y = p[1];
        result.z = p[2];
        result.tilt = o.tilt * 180.0/M_PI;
        result.pan = o.pan * 180.0/M_PI;
        result.roll = o.roll * 180.0/M_PI;
        result.fov_v = sample.fov;
        result.view_orientation = mc->view_orientation;
    }
}


//-----------------------------------------------------------------------------
//
//  CalibrationSampleArgs
//
//-----------------------------------------------------------------------------

CalibrationSampleArgs::CalibrationSampleArgs()
{
    // Rozsahy
    rangeRoll = std::pair<float,float>(-2, 2);
    rangeFOV = std::pair<float,float>(15, 35);

    // Nothing yet
    playfieldModel = nullptr;
}


HomographyGT::HomographyGT() :
    position(0,0,0),
    orientation(0,0,0),
    fov_v(0), fov_h(0.0)
{
}

HomographyGT::HomographyGT(const HomographyGT &v) :
    position(v.position),
    orientation(v.orientation),
    fov_v(v.fov_v), fov_h(v.fov_h)
{
}

HomographyGT &HomographyGT::operator=(HomographyGT &v)
{
    position = v.position;
    orientation = v.orientation;
    fov_v = v.fov_v;
    fov_h = v.fov_h;
    return *this;
}


//-----------------------------------------------------------------------------
//
//  CalibrationSample
//
//-----------------------------------------------------------------------------

CalibrationSample::CalibrationSample() :
    roll(0.0f),
    fov(45.0f),
    camera_target(0.0f, 0.0f, 0.0f)
{
}

CalibrationSample::CalibrationSample(const CalibrationSample &v) :
    roll(v.roll),
    fov(v.fov),
    camera_target(v.camera_target)
{
}

CalibrationSample &CalibrationSample::operator =(CalibrationSample v)
{
    roll = v.roll;
    fov = v.fov;
    camera_target = v.camera_target;
    return *this;
}

CalibrationSample CalibrationSample::Random(
    const CalibrationSampleArgs &args,
    cv::Vec3d camera_pos
    )
{
    CalibrationSample result;

    // view
    result.roll         = uniform(args.rangeRoll);
    result.fov          = uniform(args.rangeFOV);

    // Camera target
    if (args.playfieldModel) {
        double L = args.playfieldModel->length / 2.0f;
        double W = args.playfieldModel->width / 2.0f;

        L = L - 6.0f;
        W = W - 4.0f;

        double lMin = 0-L;
        double lMax = 0+L;
        double camlMin = camera_pos[0] - L;
        double camlMax = camera_pos[0] + L;
        double range_L_min = MAX(lMin, camlMin);
        double range_L_max = MIN(lMax, camlMax);

        std::pair<double,double> LENGTH_ARGS = std::make_pair<double,double>(
            (double)range_L_min, (double)range_L_max
            );
        std::pair<double,double> WIDTH_ARGS = std::make_pair<double,double>(
            (double)-W,(double)W
            );

        result.camera_target = cv::Vec3d(
            uniform(LENGTH_ARGS),
            uniform(WIDTH_ARGS),
            0.0
            );
    }

    return result;
}


//-----------------------------------------------------------------------------
//
//  CropArgs
//
//-----------------------------------------------------------------------------

CropArgs::CropArgs()
{
    // Rozsahy
    rangePan = std::pair<float,float>(-40, 40);
    rangeTilt = std::pair<float,float>(-25, -2);
    rangeRoll = std::pair<float,float>(-2, 2);
    rangeFOV = std::pair<float,float>(15, 35);

    // Distortion
    useDistortion = true;

    // Distortion
    rangeK1 = std::pair<float,float>(-0.45, 0.12);
    rangeK2 = std::pair<float,float>(0.0, 0.02);

    useColorAdjust = true;
}

//-----------------------------------------------------------------------------
//
//  CropSample
//
//-----------------------------------------------------------------------------


CropSample::CropSample() :
    p(0), t(0), r(0), fov(45),
    k1(0), k2(0),
    gamma(1.0),
    h(0.0), s(1.0), v(1.0)
{
}

CropSample::CropSample(const CropSample &av):
    p(av.p), t(av.t), r(av.r), fov(av.fov),
    k1(av.k1), k2(av.k2),
    gamma(av.gamma),
    h(av.h), s(av.s), v(av.v)
{

}

CropSample &CropSample::operator =(CropSample v)
{
    p = v.p; t = v.t; r = v.r; fov = v.fov;
    k1 = v.k1; k2 = v.k2;
    gamma = v.gamma;
    h = v.h; s = v.s; this->v = v.v;
    return *this;
}

float k2Fromk1(float k1)
{
    return 0.019*k1 + 0.805*k1*k1;
}

QVector4D getInverseK(QVector4D k)
{
    float _k[] = { k.x(), k.y(), k.z(), k.w(), 0, 0, 0, 0 };
    float b[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    invertCoeffs_Eigen(_k, b, 4);

    return QVector4D(b[0], b[1], b[2], b[3]);
}


QVector4D getInverseK(float k1, float k2)
{
    float k[] = { k1, k2, 0, 0, 0, 0, 0, 0 };
    float b[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    //invertCoeffs(k, b, 4);
    invertCoeffs_Eigen(k, b, 4);

    return QVector4D(b[0], b[1], b[2], b[3]);
}



std::vector<float> _distort(std::vector<float> &values, float *k)
{
    std::vector<float> result;
    result.resize(values.size());

    for (int i=0; i<(int)values.size(); i++) {

        double x = values[i];
        double x2 = x*x;
        double x4 = x2*x2;
        double d = (1.0 + k[0]*x2 + k[1]*x4 + k[2]*x4*x2 + k[3]*x4*x4);

        result[i] =x*d;
    }

    return result;
}

std::vector<float> _linspace(float vmin, float vmax, int n)
{
    std::vector<float> result;
    result.resize(n);

    if (n == 0) {
        result[0] = vmin;
    } else {
        for (int i=0; i<n; i++) {
            result[i] = vmin + (vmax - vmin)*i / (n-1);
        }
    }

    return result;
}

void _findX(std::vector<float> &vx, float x, int *iLow, float *aRelative)
{
    for (int n=0; n<(int)vx.size()-1; n++) {

        float vLow = vx[n];
        float vHi = vx[n+1];

        // Nasli sme!
        if (x >= vLow && x <= vHi) {

            (*iLow) = n;
            (*aRelative) = (x - vLow) / (vHi - vLow);
            return ;
        }
    }

    (*iLow) = 0;
    (*aRelative) = 0.0;
}

void _findMinMax(std::vector<float> &v, float *amin, float *amax)
{
    (*amin) = v[0];
    (*amax) = v[0];
    for (int i=0; i<(int)v.size(); i++) {
        if (v[i] < (*amin)) (*amin) = v[i];
        if (v[i] > (*amax)) (*amax) = v[i];
    }
}

std::vector<float> _inverseInterpolate(std::vector<float> &vx, std::vector<float>&vy, std::vector<float> &y)
{
    std::vector<float> result;
    result.resize(y.size());

    for (int i=0; i<(int)y.size(); i++) {

        int idx = 0;
        float relative = 0.0;
        _findX(vy, y[i], &idx, &relative);

        float xLo = vx[idx];
        float xHi = vx[idx+1];
        float x = xLo + (xHi-xLo)*relative;

        // vratime spocitanu interpolovanu hodnotu
        result[i] = x;
    }


    return result;
}

void invertCoeffs_Eigen(float *k, float *b, int n)
{
#if 0
    // 1. spravime linear space
    auto r = _linspace(0, 1.0, 100);
    auto dr = _distort(r, k);

    // 2. ideme si nasamplovat tvar distortion funkcia
    float dmin=0, dmax=0;
    _findMinMax(dr, &dmin, &dmax);
    auto drInterval = _linspace(dmax/2.0, dmax, n);
    auto rInterval = _inverseInterpolate(r, dr, drInterval);


    // 3. zlozime velku maticu R
    Eigen::MatrixX<float>    R(n,n);
    for (int i=0; i<n; i++) {
        for (int j=0; j<n; j++) {
            R(j,i) = pow( drInterval[i], 2*(j+1));
        }
    }

    // 4. Dstar
    Eigen::MatrixX<float>   Dstar(1, n);
    for (int i=0; i<n; i++) {
        Dstar(0,i) = (rInterval[i] / drInterval[i]) - 1.0;
    }

    // 5. Rinv
    auto Kinv = Dstar * R.inverse();
    for (int i=0; i<n; i++) {
        b[i] = Kinv(0, i);
    }
#endif
}







void invertCoeffs(float *k, float *b, int n)
{
    b[0] = -k[0];
    b[1] = 3*pow(k[0], 2.0) - k[1];

    if (n > 2) {
        b[2] = -12*pow(k[0],3) + 8*k[0]*k[1] - 1*k[2];
        b[3] = 55*pow(k[0],4) - 55*(k[0]*k[0]*k[1]) + 10*(k[0]*k[2]) + 5*(k[1]*k[1]) - 1*k[3];
        if (n > 4) {
            b[4] =  -273*pow(k[0],5) + 364*(pow(k[0],3)*k[1])
                    - 78*k[0]*k[1]*k[1] - 78*k[0]*k[0]*k[2]
                    + 12*k[1]*k[2] + 12*k[0]*k[3];
            b[5] = 1428*pow(k[0],6) - 2380*(pow(k[0],4)*k[1])
                    + 840*(k[0]*k[0]*k[1]*k[1]) - 35*pow(k[1],3)
                    + 560*(pow(k[0],3)*k[2]) - 210*(k[0]*k[1]*k[2])
                    + 7*(k[2]*k[2]) - 105*k[0]*k[0]*k[3]
                    + 14*k[1]*k[3];
        }
    }
}

const std::pair<float,float> GAMMA_ARGS = std::make_pair<float,float>(0.8, 1.2);
const std::pair<float,float> H_ARGS = std::make_pair<float,float>(0.0, 0.05);
const std::pair<float,float> S_ARGS = std::make_pair<float,float>(1.0, 0.05);
const std::pair<float,float> V_ARGS = std::make_pair<float,float>(1.0, 0.05);



CropSample CropSample::Random(const CropArgs &args)
{
    CropSample      result;

    // view
    result.p 		= uniform(args.rangePan);
    result.t 		= uniform(args.rangeTilt);
    result.r 		= uniform(args.rangeRoll);
    result.fov 		= uniform(args.rangeFOV);

    // distortion
    if (args.useDistortion) {
        result.k1 		= uniform(args.rangeK1);
        result.k2 		= k2Fromk1(result.k1) + normal(args.rangeK2);
    }

    if (args.useColorAdjust) {
        result.gamma = uniform(GAMMA_ARGS);
        result.h = normal(H_ARGS);
        result.s = normal(S_ARGS);
        result.v = normal(V_ARGS);
    } else {
        result.gamma = 1.0;
        result.h = 0.0;
        result.s = 1.0;
        result.v = 1.0;
    }

    return result;
}

//-----------------------------------------------------------------------------
//
//  PlayfieldGridImages
//
//-----------------------------------------------------------------------------

PlayfieldGridImages::PlayfieldGridImages()
{

}

PlayfieldGridImages::PlayfieldGridImages(const PlayfieldGridImages &v)
{
    images.append(v.images);
}

PlayfieldGridImages &PlayfieldGridImages::operator =(PlayfieldGridImages v)
{
    images.clear();
    images.append(v.images);
    return *this;
}

QSharedPointer<PlayfieldGridImages> PlayfieldGridImages::FromFolder(QString path)
{
    QSharedPointer<PlayfieldGridImages> result =
        makeNew<PlayfieldGridImages>();

    QList<QString>      image_paths;
    QDirIterator        it(path, QDir::Files | QDir::NoDotAndDotDot, QDirIterator::NoIteratorFlags);
    while (it.hasNext()) {
        image_paths.push_back(it.next());
    }
    image_paths.sort();

    // Load all images
    for (int i=0; i<image_paths.count(); i++) {
        auto qi = makeNew<QImage>(image_paths[i]);
        result->images.push_back(qi);
    }

    return result;
}




//-----------------------------------------------------------------------------
//
//  BaseExportProcessState
//
//-----------------------------------------------------------------------------

BaseExportProcessState::BaseExportProcessState(int atotal) :
    isAborting(false),
    totalCount(atotal),
    progress(0)
{

}

void BaseExportProcessState::abort()
{
    isAborting = true;
}

void BaseExportProcessState::next()
{
    QMutexLocker    l(&lock);

    progress += 1;
}

void BaseExportProcessState::getProgress(int &aprogress, int &atotal)
{
    QMutexLocker    l(&lock);

    aprogress = progress;
    atotal = totalCount;
}

//-----------------------------------------------------------------------------
//
//  ExportProcessState
//
//-----------------------------------------------------------------------------

ExportProcessState::ExportProcessState(int atotal) :
    BaseExportProcessState(atotal)
{
}

void ExportProcessState::setCurrentFrame(QSharedPointer<RenderedImage> frame)
{
    QMutexLocker    l(&lock);
    currentFrame = frame;
}


//-----------------------------------------------------------------------------
//
//  ExportCalibrationProcessState
//
//-----------------------------------------------------------------------------

ExportCalibrationProcessState::ExportCalibrationProcessState(int atotal) :
    BaseExportProcessState(atotal)
{
}

void ExportCalibrationProcessState::setCurrentFrame(
    QSharedPointer<RenderedCalibrationImage> frame
    )
{
    QMutexLocker    l(&lock);
    currentFrame = frame;
}



//-----------------------------------------------------------------------------
//
//  CropRenderer
//
//-----------------------------------------------------------------------------


CropRenderer::CropRenderer(
        QOffscreenSurface *asurface,
        QSize asize
        ) :
    size(asize),
    context(nullptr),
    surface(asurface),
    program(nullptr),
    rt(asize),
    texDistort(asize),
    isInitialized(false)
{
}

CropRenderer::~CropRenderer()
{
    qDebug() << "Renderer destroyed";
    if (isInitialized) {

        if (context) context->makeCurrent(surface);

        textures.clear();
        texDistort.clear();

        if (context) {
            context->doneCurrent();
            delete context;
            context = nullptr;
        }

        if (program) {
            program->destroy();
            delete program;
            program = nullptr;
        }

    }
}

bool CropRenderer::initialize()
{
    // zrobime novy kontext
    context = new QOpenGLContext();
    context->setFormat(surface->format());
    context->create();
    context->makeCurrent(surface);
    context->functions()->initializeOpenGLFunctions();

    // Loadneme shaders
    program = new PinholeProgram(context->functions());
    program->init();

    auto f = context->functions();
    rt.init(f);

    return true;
}


static float distort(QVector4D k, float r)
{
    float r2 = r*r;
    float r4 = r2*r2;
    double d = 1.0 + k.x()*r2 + k.y()*r4 + k.z()*(r2*r4) + k.w()*(r4*r4);
    return d;
}

static QPointF distortPoint(
                    QPointF c, float x, float y, QVector4D k, float f,
                    float maxR, bool *valid
                )
{
    double rx = (x - c.x()) / (double)f;
    double ry = (y - c.y()) / (double)f;
    double r2 = (rx*rx + ry*ry);
    double r4 = r2*r2;
    double r8 = r4*r4;

    // distort ratio
    double d = 1.0
            + k.x()*r2         + k.y()*r4
            + k.z()*(r2*r4)    + k.w()*(r8)
            ;

    double _x = c.x() + rx*d*f;
    double _y = c.y() + ry*d*f;

    double r = sqrt(r2);
    (*valid) = (r <= maxR);
    return QPointF(_x, _y);
}

static float getMaxR(int width, int height)
{
    float aspect = (float)width / (float)height;
    float h = 0.5;
    float w = h * aspect;
    return sqrt(h*h + w*w);
}

QSharedPointer<RenderedImage> CropRenderer::render(
            QSharedPointer<Image> image,
            CropSample sample
        )
{

    if (!isInitialized) {
        isInitialized = initialize();
        // Error ?
        if (!isInitialized) return nullptr;
    }

    context->makeCurrent(surface);
    auto f = context->functions();

    //-----------------------------------------------
    //  View parameters & distortion function

    QVector4D       k(sample.k1, sample.k2, 0, 0);
    QOpenGLTexture  *texd = this->texDistort.get(k, dist);


    //-----------------------------------------------
    //  Rendering

    rt.bind(f);

    f->glClearColor(0.5f, 0.0f, 0.0f, 1.0f);
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    f->glViewport(0,0,size.width(),size.height());

    float       imw = 1.0;
    float       imh = 1.0;

    // Loadujeme obrazok
    QOpenGLTexture *texture = textures.get("main", image);

    // odlozime si rozlisko
    if (image) {
        imw = image->image.width();
        imh = image->image.height();
    }

    // Kreslime pohlad
    program->bind();

        // nahodime texturu
        if (texture) {
            texture->bind(0);
            program->setTexture(0);
        }
        if (texd) {
            texd->bind(2);
            program->setDistortTexture(2);
        }

        program->prepareView(sample, size.width(), size.height());
        program->setArgs(sample.gamma, QVector3D(sample.h, sample.s, sample.v));
        program->setK(k);
        program->draw();

    program->unbind();


    // Vratime vysledok
    auto result = makeNew<RenderedImage>();

    // Download result
    rt.download(f, result->image);
    rt.unbind(f);

    result->mapping = image->mapping;
    //result->RK_inverse = program->getInverseRK(sample, size.width(), size.height());
    result->labelVolume = reproject(image->mapping, result->RK_inverse, k, imw, imh);

    context->doneCurrent();

    return result;
}

cv::Mat CropRenderer::reproject(
            QSharedPointer<Data::ImageMapping> mapping,
            cv::Mat rk_inv, QVector4D k,
            float imw, float imh
        )
{
    int         n = mapping->kps.size();
    float       outw = (float)size.width();
    float       outh = (float)size.height();

    // Maximalny povoleny radius
    float       distMaxR = dist.xRange().second;
    QPointF     c(outw/2.0, outh/2.0);

    double      fPX;
    if (program->HeightWise()) {
        fPX = outh;
    } else {
        fPX = outw;
    }

    cv::Mat     result = cv::Mat_<double>(3, n);
    if (mapping) {

        cv::Mat     r3d, pi;
        double      x, y, z, v;
        double      theta, phi;

        for (int i=0; i<n; i++) {
            Data::KeyPointMapping &src = mapping->kps[i];

            // Angles in panorama
            theta = (2.0*M_PI * src.x / imw) - M_PI;
            phi   =     M_PI * src.y / imh - M_PI_2;

            r3d = (cv::Mat_<double>(3,1) <<
                    sin(phi) * sin(theta),
                    cos(phi),
                    sin(phi) * cos(theta)
            );

            pi = rk_inv * r3d;
            x = pi.at<double>(0, 0);
            y = pi.at<double>(1, 0);
            z = pi.at<double>(2, 0);


            // visibility ?
            if (z > 0) {
                x = (x/z) * (outh / outw);  //  -> do <0;1> rozsahu
                y = (y/z) * (outh / outh);  //  -> do <0;1> rozsahu
                v = 1.0;

                /*
                // Ak sme mimo rozsah, tiez bod neplati
                if ((x < 0) || (x > 1) ||
                    (y < 0) || (y > 1)
                        ) {

                    x = 0.0;
                    y = 0.0;
                    v = 0.0;
                }
                */

            } else {
                x = 0.0;
                y = 0.0;
                v = 0.0;
            }

            if (v > 0.0) {
                // distortion
                bool valid = true;
                QPointF dp = distortPoint(
                                    c, x*outw, y*outh,
                                    k, fPX, distMaxR, &valid
                                );
                if (valid) {
                    x = dp.x() / outw;
                    y = dp.y() / outh;
                } else {
                    x = 0.0;
                    y = 0.0;
                    v = 0.0;
                }
            }

            // Ukladame stlpcovy vektor do matice
            result.at<double>(0, i) = x;
            result.at<double>(1, i) = y;
            result.at<double>(2, i) = v;
        }
    }

    return result;
}

//-----------------------------------------------------------------------------
//
//  RenderTarget
//
//-----------------------------------------------------------------------------

RenderTarget::RenderTarget(QSize asize) :
    size(asize),
    pixels(nullptr)
{
    // Naalokujeme data
    pixels = (uchar*)malloc(size.width() * size.height() * 3);
}

RenderTarget::~RenderTarget()
{
    if (pixels) {
        free(pixels);
        pixels = nullptr;
    }
}

void RenderTarget::init(QOpenGLFunctions *f)
{
    // Render Target
    f->glGenRenderbuffers(1, &rtTarget);
    f->glBindRenderbuffer(GL_RENDERBUFFER, rtTarget);
    f->glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, size.width(), size.height());
    f->glBindRenderbuffer(GL_RENDERBUFFER, 0);

    // Depth Buffer
    f->glGenRenderbuffers(1, &dsTarget);
    f->glBindRenderbuffer(GL_RENDERBUFFER, dsTarget);
    f->glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, size.width(), size.height());
    f->glBindRenderbuffer(GL_RENDERBUFFER, 0);

    // Framebuffer object
    f->glGenFramebuffers(1, &fbo);
    f->glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    f->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rtTarget);
    f->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, dsTarget);
    f->glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void RenderTarget::bind(QOpenGLFunctions *f)
{
    QOpenGLExtraFunctions *fex = (QOpenGLExtraFunctions*)f;
    f->glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    int buf = GL_COLOR_ATTACHMENT0;
    fex->glDrawBuffers(1, (const GLenum*)&buf); //GL_COLOR_ATTACHMENT0);
}

void RenderTarget::unbind(QOpenGLFunctions *f)
{
    // Clean up
    f->glBindRenderbuffer(GL_RENDERBUFFER, 0);
    f->glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void RenderTarget::download(QOpenGLFunctions *f, QImage &result)
{
    QOpenGLExtraFunctions *fex = (QOpenGLExtraFunctions*)f;

    // Download from GPU
    fex->glReadBuffer(GL_COLOR_ATTACHMENT0);
    fex->glReadPixels(0,0, size.width(), size.height(),
                 GLenum(GL_RGB), GLenum(GL_UNSIGNED_BYTE),
                 pixels
                 );

    // Copy to result image
    result = QImage(size.width(), size.height(), QImage::Format_RGB888);
    // Nakopcime data
    int h=size.height();
    for (int y=0; y<h; y++) {
        uchar *dst = result.bits() + y*(size.width() * 3);
        uchar *src = pixels + (h-1-y)*(size.width() * 3);
        // copy line
        memcpy(dst, src, size.width() * 3);
    }
}


//-----------------------------------------------------------------------------
//
//  TextureCache
//
//-----------------------------------------------------------------------------

TextureCache::TextureCache()
{

}

TextureCache::~TextureCache()
{
    clear();
}

void TextureCache::clear()
{
    QMapIterator<QString, TextureCache::Item>    it(items);
    while (it.hasNext()) {
        it.next();
        auto item = it.value();
        QOpenGLTexture  *texture = item.texture;
        if (texture) {
            delete texture;
        }
    }

    items.clear();
}

QOpenGLTexture *TextureCache::get(QString key, QSharedPointer<Image> image)
{
    QOpenGLTexture *result = nullptr;

    if (items.contains(key)) {

        TextureCache::Item &item = items[key];
        if (item.image == image) {
            return item.texture;
        }
        // Done with old texture
        if (item.texture) {
            delete item.texture;
            item.texture = nullptr;
        }

        // Make new texture
        item.image = image;
        item.texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
        item.texture->setData(image->image, QOpenGLTexture::DontGenerateMipMaps);
        item.texture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        item.texture->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
        item.texture->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);
        result = item.texture;

    } else {

        // Insert
        TextureCache::Item  item;
        // Make new texture
        item.image = image;
        item.texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
        item.texture->setData(image->image, QOpenGLTexture::DontGenerateMipMaps);
        item.texture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        item.texture->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
        item.texture->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);
        result = item.texture;

        // Store the new value
        items[key] = item;
    }

    return result;
}

QOpenGLTexture *TextureCache::get(QString key, QSharedPointer<QImage> image)
{
    QOpenGLTexture *result = nullptr;

    if (items.contains(key)) {

        TextureCache::Item &item = items[key];
        if (item.qimage == image) {
            return item.texture;
        }
        // Done with old texture
        if (item.texture) {
            delete item.texture;
            item.texture = nullptr;
        }

        // Make new texture
        item.qimage = image;
        item.texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
        item.texture->setData(*image.data(), QOpenGLTexture::DontGenerateMipMaps);
        item.texture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        item.texture->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
        item.texture->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);
        result = item.texture;

    } else {

        // Insert
        TextureCache::Item  item;
        // Make new texture
        item.qimage = image;
        item.texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
        item.texture->setData(*image.data(), QOpenGLTexture::DontGenerateMipMaps);
        item.texture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        item.texture->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
        item.texture->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);
        result = item.texture;

        // Store the new value
        items[key] = item;
    }

    return result;
}

//-----------------------------------------------------------------------------
//
//  DistortionTexture
//
//-----------------------------------------------------------------------------

DistortionTexture::DistortionTexture(QSize asize) :
    size(asize),
    texture(nullptr)
{

}

DistortionTexture::~DistortionTexture()
{
    clear();
}

void DistortionTexture::clear()
{
    if (texture) {
        delete texture;
        texture = nullptr;
    }
}

QOpenGLTexture *DistortionTexture::get(QVector4D k, InterpolatedFunction &dist)
{

    float maxR = getMaxR(size.width(), size.height());
    int n = 256;
    dist.reset();
    for (int i=0; i<n; i++) {
        float r = 0.0 + 2.0*maxR*(float)i/(float)(n-1);
        float d = r * distort(k, r);
        dist.add(r, d);
    }

    computeDistortTexture(dist, maxR, true);

    return texture;
}

void DistortionTexture::computeDistortTexture(InterpolatedFunction &func, float maxR, bool inverse)
{
    // Vypocitame maximalny diagonalny radius
    int n = 256;

    if (!texture) {
        texture = new QOpenGLTexture(QOpenGLTexture::Target1D);
        texture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);

        texture->setSize(n);
        texture->setFormat(QOpenGLTexture::R32F);
        texture->allocateStorage(QOpenGLTexture::Red, QOpenGLTexture::Float32);
    }


    // Spocitame nove hodnoty
    std::vector<float>      f;
    f.resize(n);
    for (int i=0; i<n; i++) {
        double y = maxR * ((float)i / (float)(n-1));
        double r = 0;
        if (inverse) {
            r = func.getX(y);
        } else {
            r = func.getY(y);
        }
        if (y > 0) {
            f[i] = r/y;
        } else {
            f[i] = 1.0;
        }
    }

    texture->setData(QOpenGLTexture::Red, QOpenGLTexture::Float32, f.data());
}


//-----------------------------------------------------------------------------
//
//  CalibrationRenderer
//
//-----------------------------------------------------------------------------

CalibrationRenderer::CalibrationRenderer(
    QOffscreenSurface *asurface,
    QSize asize
    ) :
    size(asize),
    context(nullptr),
    surface(asurface),
    program(nullptr),
    playfieldProgram(nullptr),
    rt(asize),
    texDistort(asize),
    isInitialized(false)
{
}

CalibrationRenderer::~CalibrationRenderer()
{
    if (isInitialized) {
        if (context) context->makeCurrent(surface);

        textures.clear();
        texDistort.clear();

        if (context) {
            context->doneCurrent();
            delete context;
            context = nullptr;
        }

        if (program) {
            program->destroy();
            delete program;
            program = nullptr;
        }

        if (playfieldProgram) {
            playfieldProgram->destroy();
            delete playfieldProgram;
            playfieldProgram = nullptr;
        }
    }
}

bool CalibrationRenderer::initialize()
{
    // zrobime novy kontext
    context = new QOpenGLContext();
    context->setFormat(surface->format());
    context->create();
    context->makeCurrent(surface);
    context->functions()->initializeOpenGLFunctions();

    auto f = context->functions();

    // Loadneme shaders
    program = new PinholeProgram(f);
    program->init();

    playfieldProgram = new PinholePlayfieldProgram(f);
    playfieldProgram->init();

    rt.init(f);
    return true;
}


QSharedPointer<RenderedCalibrationImage> CalibrationRenderer::render(
    QSharedPointer<Image> image,
    QSharedPointer<PlayfieldGridImages> gridImages,
    CalibrationCameraPose pose
    )
{
    if (!isInitialized) {
        isInitialized = initialize();
        // Error ?
        if (!isInitialized) return nullptr;
    }

    // Vratime vysledok
    auto result = makeNew<RenderedCalibrationImage>();

    context->makeCurrent(surface);

    renderMain(image, pose, result);
    renderGrid(pose, gridImages, result);

    // compute horizontal FOV
    result->fov_v = pose.fov_v;

    double  fh = fov_to_f(pose.fov_v * M_PI/180.0);
    double  aspect = (double)size.width() / (double)size.height();

    // to degrees
    result->fov_h = 2.0 * atan2(aspect, 2.0*fh) * 180.0/M_PI;

    context->doneCurrent();
    return result;
}


void CalibrationRenderer::renderMain(
    QSharedPointer<Image> image,
    CalibrationCameraPose pose,
    QSharedPointer<RenderedCalibrationImage> result
    )
{
    auto f = context->functions();

    // Texture & Distortion
    auto texture = textures.get("main", image);

    QVector4D       k(0,0,0,0);
    auto texd = this->texDistort.get(k, dist);

    //-----------------------------------------------
    //  Rendering

    rt.bind(f);

    f->glClearColor(0.5f, 0.0f, 0.0f, 1.0f);
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    f->glViewport(0,0,size.width(),size.height());

    // Kreslime pohlad
    program->bind();
    // nahodime texturu
    if (texture) {
        texture->bind(0);
        program->setTexture(0);
    }
    if (texd) {
        texd->bind(2);
        program->setDistortTexture(2);
    }

    CropSample          sample;
    sample.t = pose.view_orientation.tilt * 180.0/M_PI;
    sample.p = pose.view_orientation.pan * 180.0/M_PI;
    sample.r = pose.view_orientation.roll * 180.0/M_PI;
    sample.fov = pose.fov_v;

    program->prepareView(sample, size.width(), size.height());
    program->setArgs(1.0, QVector3D(0, 1, 1));
    program->setK(k);
    program->draw();

    program->unbind();

    // Download result
    rt.download(f, result->imagePlayfield);
    rt.unbind(f);
}

void CalibrationRenderer::renderGrid(
    CalibrationCameraPose pose,
    QSharedPointer<PlayfieldGridImages> gridImages,
    QSharedPointer<RenderedCalibrationImage> result
    )
{
    auto f = context->functions();

    for (int i=0; i<gridImages->images.count(); i++) {
        QString key = QString::asprintf("grid-%d", i);
        auto texture = textures.get(key, gridImages->images[i]);

        //-----------------------------------------------
        //  Rendering

        rt.bind(f);

        f->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        f->glViewport(0,0,size.width(),size.height());

        playfieldProgram->bind();

        if (texture) {
            texture->bind(0);
            playfieldProgram->setTexture(0);
        }

        playfieldProgram->prepareView(
            _toQ(pose.mv),
            pose.fov_v,
            (double)size.width() / (double)size.height()
            );
        playfieldProgram->draw();
        playfieldProgram->unbind();

        // Download result
        QImage      image;
        rt.download(f, image);
        rt.unbind(f);
        result->gridImages.push_back(image);
    }
}


//-----------------------------------------------------------------------------
//
//  PinholePlayfieldProgram
//
//-----------------------------------------------------------------------------

PinholePlayfieldProgram::PinholePlayfieldProgram(
    QOpenGLFunctions *func
    ) :
    f(func),
    program(nullptr),
    posVertex(0),
    posTex(0),
    posTexture(0),
    posMVP(0)
{
    heightWise = true;

    // Conversion to OpenGL convention
    toGL = _toQ(_toGL());
}

void PinholePlayfieldProgram::init()
{
    // Setup program
    program = new QOpenGLShaderProgram();
    program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/Shaders/playfield.vert");
    program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/Shaders/playfield.frag");
    program->link();

    posVertex = program->attributeLocation("vertex");
    posTex = program->attributeLocation("tex");
    posTexture = program->uniformLocation("texture");
    posMVP = program->uniformLocation("MVP");
}

void PinholePlayfieldProgram::destroy()
{
    if (program) {
        delete program;
        program = nullptr;
    }
}

void PinholePlayfieldProgram::bind()
{
    program->bind();
}

void PinholePlayfieldProgram::unbind()
{

    program->release();
}


void PinholePlayfieldProgram::prepareView(
        QMatrix4x4 view,
        float fov_v_deg, double aspect
    )
{    
    QMatrix4x4      model;
    model.setToIdentity();

    // Projection matrix
    QMatrix4x4 projection;
    projection.perspective(fov_v_deg, aspect, 0.1f, 500.0f);


    QMatrix4x4       MVP = projection * toGL * view * model;
    program->setUniformValue(posMVP, MVP);

}

void PinholePlayfieldProgram::setTexture(GLint value)
{
    program->setUniformValue(posTexture, value);
}


const float LENGTH_DIV_2 = 105.0f / 2.0f;
const float WIDTH_DIV_2 = 68.0f / 2.0f;

void PinholePlayfieldProgram::draw()
{

    // Vertices
#if 0
    static const GLfloat vertices[] = {
        -LENGTH_DIV_2, 0, WIDTH_DIV_2,
        -LENGTH_DIV_2, 0, -WIDTH_DIV_2,
        LENGTH_DIV_2, 0,  WIDTH_DIV_2,

        -LENGTH_DIV_2, 0, -WIDTH_DIV_2,
        LENGTH_DIV_2, 0,  WIDTH_DIV_2,
        LENGTH_DIV_2, 0,  -WIDTH_DIV_2
    };
#else
    static const GLfloat vertices[] = {
        -LENGTH_DIV_2, -WIDTH_DIV_2, 0,
        -LENGTH_DIV_2, WIDTH_DIV_2, 0,
        LENGTH_DIV_2, -WIDTH_DIV_2, 0,

        -LENGTH_DIV_2, WIDTH_DIV_2, 0,
        LENGTH_DIV_2, -WIDTH_DIV_2, 0,
        LENGTH_DIV_2, WIDTH_DIV_2, 0
    };
#endif

    static const GLfloat tex[] = {
        0.0, 0.0,
        0.0, 1.0,
        1.0, 0.0,

        0.0, 1.0,
        1.0, 0.0,
        1.0, 1.0
    };

    f->glEnable(GL_BLEND);
    //f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //f->glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    f->glBlendFunc(GL_ONE, GL_ONE);

    // Bind vertices
    f->glVertexAttribPointer(posVertex, 3, GL_FLOAT, GL_FALSE, 0, vertices);
    f->glVertexAttribPointer(posTex, 2, GL_FLOAT, GL_FALSE, 0, tex);
    f->glEnableVertexAttribArray(posVertex);
    f->glEnableVertexAttribArray(posTex);

    // Draw
    f->glDrawArrays(GL_TRIANGLES, 0, 6);

    // cleanup
    f->glDisableVertexAttribArray(posVertex);
    f->glDisableVertexAttribArray(posTex);

    f->glDisable(GL_BLEND);

}


//-----------------------------------------------------------------------------
//
//  PinholeProgram
//
//-----------------------------------------------------------------------------

PinholeProgram::PinholeProgram(QOpenGLFunctions *func) :
    f(func),
    program(nullptr),
    posVertex(0),
    posTex(0),
    posTexture(0),
    posDistortTexture(0),
    posCanvas(0),
    posRK(0),
    posArgs(0),
    posK(0)
{
    heightWise = true;
}

void PinholeProgram::init()
{
    // Setup program
    program = new QOpenGLShaderProgram();
    program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/Shaders/default.vert");
    program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/Shaders/default.frag");
    if (!program->link()) {
        qDebug() << program->log();
    } else {
        qDebug() << "Link pinhole successful";
    }

    posVertex = program->attributeLocation("vertex");
    posTex = program->attributeLocation("tex");
    posTexture = program->uniformLocation("texture");
    posDistortTexture = program->uniformLocation("textureDistort");
    posRK = program->uniformLocation("rk");
    posCanvas = program->uniformLocation("canvas");
    posArgs = program->uniformLocation("args");
    posK = program->uniformLocation("k");
}

void PinholeProgram::destroy()
{
    if (program) {
        delete program;
        program = nullptr;
    }
}

void PinholeProgram::bind()
{
    program->bind();
}

void PinholeProgram::unbind()
{

    program->release();
}

void PinholeProgram::setCanvas(QVector2D value)
{
    program->setUniformValue(posCanvas, value);
}

void PinholeProgram::setTexture(GLint value)
{
    program->setUniformValue(posTexture, value);
}

void PinholeProgram::setDistortTexture(GLint value)
{
    program->setUniformValue(posDistortTexture, value);
}

void PinholeProgram::setRK(QMatrix3x3 value)
{
    program->setUniformValue(posRK, value);
}

void PinholeProgram::setArgs(float gamma, QVector3D hsv)
{
    QVector4D   args(hsv, gamma);
    program->setUniformValue(posArgs, args);
}

void PinholeProgram::setK(QVector4D k)
{
    program->setUniformValue(posK, k);
}

void PinholeProgram::draw()
{
    // Vertices
    static const GLfloat vertices[] = {
        -1.0, 1.0,
         1.0, 1.0,
         1.0, -1.0,

        -1.0, 1.0,
        1.0, -1.0,
        -1.0, -1.0
    };
    static const GLfloat tex[] = {
        0.0, 0.0,
        1.0, 0.0,
        1.0, 1.0,

        0.0, 0.0,
        1.0, 1.0,
        0.0, 1.0
    };

    // Bind vertices
    f->glVertexAttribPointer(posVertex, 2, GL_FLOAT, GL_FALSE, 0, vertices);
    f->glVertexAttribPointer(posTex, 2, GL_FLOAT, GL_FALSE, 0, tex);
    f->glEnableVertexAttribArray(posVertex);
    f->glEnableVertexAttribArray(posTex);

    // Draw
    f->glDrawArrays(GL_TRIANGLES, 0, 6);

    // cleanup
    f->glDisableVertexAttribArray(posVertex);
    f->glDisableVertexAttribArray(posTex);
}



void PinholeProgram::prepareView(CropSample s, int width, int height)
{
    // Canvas
    QVector2D       _canvas(1.0, 1.0);
    if (height > 0) _canvas.setX((float)width / (float)height);
    setCanvas(_canvas);

    // Camera Intrinsic, Rotation
    Orientation     o(cv::Vec3d(s.p, s.t, s.r) * M_PI/180.0);
    cv::Mat         R0 = _extract_rotation_from_4x4(_rx(90 * M_PI/180.0));
    cv::Mat         R = _extract_rotation_from_4x4(o.rotation_zxz());
    double          f = fov_to_f(s.fov * M_PI/180.0);
    cv::Mat         K = (cv::Mat_<double>(3,3) <<
             f, 0, 0,
             0, f, 0,
             0, 0, 1.0
         );

    //cv::Mat			RK = R0*R * K.inv();

    cv::Mat RK = K * R * R0;
    RK = RK.inv();
    RK.convertTo(RK, CV_32F);

    QMatrix3x3      _rk((const float*)RK.data);
    setRK(_rk);
}


//-----------------------------------------------------------------------------
//
//  DatasetSink
//
//-----------------------------------------------------------------------------

DatasetSink::DatasetSink(
        QString apath, int acount, int asw, int ash,
        bool alabelimages
        ) :
    path(apath),
    totalCount(acount),
    scaleSize(cv::Size(asw, ash)),
    index(0),
    writeLabelImages(alabelimages)
{
    // spocitame padding
    padding = QString::number(acount+1).length();
    imagePrefix = getBestPrefix();

    // vyrabame folder aj subfolders
    QDir        dir(path);
    dir.mkpath(".");
    dir.mkpath("images");
    dir.mkpath("labels");

    // Otvorime subor
    labelsCSV = new QFile(path + "/labels.csv");
    if (labelsCSV->open(QFile::WriteOnly | QFile::Text | QFile::Append)) {

        // Neprepiseme, ale dopisujeme na koniec
        auto size = labelsCSV->size();
        labelsCSV->seek(size);

        // sme OK
    } else {
        delete labelsCSV;
        labelsCSV = nullptr;
    }
}

DatasetSink::~DatasetSink()
{
    if (labelsCSV) {
        labelsCSV->close();
        delete labelsCSV;
        labelsCSV = nullptr;
    }
}

QString DatasetSink::getBestPrefix()
{
    int         counter = 0;
    while (true) {

        QString base = "image-" + QString::number(counter).rightJustified(3, '0');
        QString fn = base + "-" + QString::number(0).rightJustified(padding, '0') + ".jpg";
        QString fullFn = path + "/images/" + fn;
        QFileInfo       fi(fullFn);
        if (!fi.exists()) {
            return base;
        }

        counter ++;
    }

    return "image";
}

QString DatasetSink::getImageFilename(int aindex)
{
    return imagePrefix + "-" + QString::number(aindex).rightJustified(padding, '0') + ".jpg";
}

QString DatasetSink::labelToString(cv::Mat volume)
{
    QString result;
    QTextStream s(&result);

    if (volume.empty()) return result;

    int m = volume.rows;
    int n = volume.cols;
    for (int x=0; x<n; x++) {
        for (int y=0; y<m; y++) {
            s << "," + QString::number(volume.at<double>(y,x));
        }
    }

    return result;
}


static cv::Mat toMat(QImage const &img, int format)
{
   //same as convert mat to qimage, the fifth parameter bytesPerLine()
   //indicate how many bytes per row
   //If you want to copy the data you need to call clone(), else QImage
   //cv::Mat will share the buffer
   return cv::Mat(img.height(), img.width(), format,
                  const_cast<uchar*>(img.bits()), img.bytesPerLine()).clone();
}


void DatasetSink::write(QSharedPointer<RenderedImage> frame, CropSample sample)
{
    QString imageFileName = getImageFilename(index);

    // Obrazok ; k1 ; k2 ; labelPoints
    QString csvRow =
            // Obrazok
            imageFileName +

            // k1; k2
            "," + QString::number(sample.k1) +
            "," + QString::number(sample.k2) +

            // labelPoints
            labelToString(frame->labelVolume) +
            "\n";

    if (labelsCSV) {
        labelsCSV->write(csvRow.toUtf8());
    }

    // este ulozime obrazok
    cv::Mat     mFrame = toMat(frame->image, CV_8UC3);
    cv::Mat     mConv;
    cv::Mat     mFinal;
    cv::cvtColor(mFrame, mConv, cv::COLOR_BGR2RGB);

    int w = mConv.cols;
    int h = mConv.rows;

    if (scaleSize.width != w || scaleSize.height != h) {
        // zoskalujeme
        cv::resize(mConv, mFinal, scaleSize, 0, 0, cv::INTER_AREA);
    } else {
        mFinal = mConv;
    }

    // Este skusime nakreslit bodky niekde
    w = mFinal.cols;
    h = mFinal.rows;

    std::vector<int>			params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);

    QString     fullFn = path + "/images/" + imageFileName;
    cv::String  fn(fullFn.toUtf8());
    cv::imwrite(fn, mFinal, params);

    if (writeLabelImages) {
        int n = frame->labelVolume.cols;
        for (int i=0; i<n; i++) {
            double  x = frame->labelVolume.at<double>(0, i);
            double  y = frame->labelVolume.at<double>(1, i);
            double  v = frame->labelVolume.at<double>(2, i);
            if (v > 0) {

                int     x0 = (x * w) - 4;
                int     x1 = (x * w) + 4;
                int     y0 = (y * h) - 4;
                int     y1 = (y * h) + 4;

                cv::line(mFinal,
                         cv::Point(x0, y0), cv::Point(x1, y1),
                         cv::Scalar(255, 0, 255), 1
                         );
                cv::line(mFinal,
                         cv::Point(x1, y0), cv::Point(x0, y1),
                         cv::Scalar(255, 0, 255), 1
                         );
            }
        }

        fullFn = path + "/labels/" + imageFileName;
        cv::imwrite(cv::String(fullFn.toUtf8()), mFinal, params);
    }

    index ++;
}


//-----------------------------------------------------------------------------
//
//  CalibrationDatasetSink
//
//-----------------------------------------------------------------------------

CalibrationDatasetSink::CalibrationDatasetSink(
        QString apath,
        int acount,
        int asw,
        int ash
        ) :
    path(apath),
    totalCount(acount),
    scaleSize(cv::Size(asw, ash)),
    index(0)
{
    QDir        dir(path);
    dir.mkpath(".");
    dir.mkpath("images");

    // Otvorime subor
    labelsCSV = new QFile(path + "/labels.csv");
    if (labelsCSV->open(QFile::WriteOnly | QFile::Text | QFile::Append)) {

        // Write CSV header
        if (labelsCSV->size() == 0) {
            QString csvRow = "name,x,y,z,tilt,pan,roll,fov_v,fov_h\n";
            labelsCSV->write(csvRow.toUtf8());
        }

        // Neprepiseme, ale dopisujeme na koniec
        auto size = labelsCSV->size();
        labelsCSV->seek(size);

        // sme OK
    } else {
        delete labelsCSV;
        labelsCSV = nullptr;
    }
}

CalibrationDatasetSink::~CalibrationDatasetSink()
{
    if (labelsCSV) {
        labelsCSV->close();
        delete labelsCSV;
        labelsCSV = nullptr;
    }
}

void CalibrationDatasetSink::write(
    QSharedPointer<RenderedCalibrationImage> frame,
    CalibrationCameraPose pose
    )
{
    // Create proper folder
    QString dirName = QString::asprintf(
        "images/%04dXXX",
        (int)(index / 1000)
        );
    QDir        dir(path);
    dir.mkpath(dirName);

    // Base name for image group
    QString baseName = getBasename(index);

    // Main image
    queue.push(get_job(frame, &frame->imagePlayfield, baseName, "-cam.png"));
    //saveResized(baseName, "-cam.png", frame->imagePlayfield);

    // All grid channels
    for (int i=0; i<frame->gridImages.count(); i++) {
        QString suffix = QString::asprintf("-lines%d.png", i);
        queue.push(get_job(frame, &frame->gridImages[i], baseName, suffix));
        //saveResized(baseName, suffix, frame->gridImages[i]);
    }

    // Write CSV label
    QString csvRow =
        baseName +
        // Camera position
        "," + QString::number(pose.x, 'g', 12) +
        "," + QString::number(pose.y, 'g', 12) +
        "," + QString::number(pose.z, 'g', 12) +

        // Camera orientation
         "," + QString::number(pose.tilt, 'g', 12) +
         "," + QString::number(pose.pan, 'g', 12) +
         "," + QString::number(pose.roll, 'g', 12) +

        // FOV
         "," + QString::number(frame->fov_v, 'g', 12) +        // fov v
         "," + QString::number(frame->fov_h, 'g', 12) +        // fov h
        "\n";

    if (labelsCSV) {
        labelsCSV->write(csvRow.toUtf8());
    }

    index ++;
}

QSharedPointer<ResizeJob> CalibrationDatasetSink::get_job(
            QSharedPointer<RenderedCalibrationImage> frame,
            QImage *aimage,
            QString base_name,
            QString suffix
        )
{
    QString     full_path = path + "/" + base_name + suffix;

    QSharedPointer<ResizeJob> result = makeNew<ResizeJob>(
        frame,
        aimage,
        full_path
    );

    return result;
}

void CalibrationDatasetSink::saveResized(QString baseName, QString suffix, QImage &image)
{
    // Convert to BGR
    cv::Mat     mFrame = toMat(image, CV_8UC3);
    cv::Mat     mConv;
    cv::Mat     mFinal;
    cv::cvtColor(mFrame, mConv, cv::COLOR_RGB2BGR);

    // Resize
    int w = mConv.cols;
    int h = mConv.rows;

    if (scaleSize.width != w || scaleSize.height != h) {
        // zoskalujeme
        cv::resize(mConv, mFinal, scaleSize, 0, 0, cv::INTER_AREA);
    } else {
        mFinal = mConv;
    }

    QString     full_path = path + "/" + baseName + suffix;
    cv::String  fn(full_path.toUtf8());
    cv::imwrite(fn, mFinal);
}


QString CalibrationDatasetSink::getBasename(int aindex)
{
    QString result = QString::asprintf(
        "images/%04dXXX/%07d",
        (int)(aindex / 1000),
        aindex
        );
    return result;
}

void CalibrationDatasetSink::start()
{
    // spustit workerov
    int num_workers = 2;

    for (int i=0; i<num_workers; i++) {
        ResizeJobWorker *worker = new ResizeJobWorker(scaleSize, &queue);
        worker->start();
        workers.push_back(worker);
    }
}

void CalibrationDatasetSink::stop()
{
    // wait for complete
    queue.abort();
    for (int i=0; i<workers.count(); i++) {
        workers[i]->wait();
    }

    while (workers.count() > 0) {
        ResizeJobWorker *w = workers.front();
        workers.pop_front();
        delete w;
    }
}


ResizeJobWorker::ResizeJobWorker(cv::Size asize, ResizeJobQueue *aqueue) :
    scaleSize(asize),
    queue(aqueue)
{
}

void ResizeJobWorker::run()
{
    while (true) {

        QSharedPointer<ResizeJob>   job = queue->get();

        if (job) {
            // work the shit

            // Convert to BGR
            cv::Mat     mFrame = toMat(*job->image, CV_8UC3);
            cv::Mat     mConv;
            cv::Mat     mFinal;
            cv::cvtColor(mFrame, mConv, cv::COLOR_RGB2BGR);

            // Resize
            int w = mConv.cols;
            int h = mConv.rows;

            if (scaleSize.width != w || scaleSize.height != h) {
                // zoskalujeme
                cv::resize(mConv, mFinal, scaleSize, 0, 0, cv::INTER_AREA);
            } else {
                mFinal = mConv;
            }

            cv::String  fn(job->filename.toUtf8());
            cv::imwrite(fn, mFinal);
        } else {
            if (queue->aborting()) break;
        }
    }
}




//-----------------------------------------------------------------------------
//
//  InterpolatedFunction class
//
//-----------------------------------------------------------------------------

InterpolatedFunction::InterpolatedFunction() :
    invertible(true)
{

}

void InterpolatedFunction::reset()
{
    invertible = true;
    data.clear();
    xMin = xMax = yMin = yMax = 0.0;
}

void InterpolatedFunction::add(float x, float y)
{
    if (data.size() == 0) {

        data.push_back(std::make_pair(x,y));

        xMin = x;
        xMax = x;
        yMin = y;
        yMax = y;

    } else {

        // prikladame len pokial je funkcia prosta
        if (invertible) {
            if (y > yMax) {
                data.push_back(std::make_pair(x,y));
                xMin = (x < xMin ? x : xMin);
                xMax = (x > xMax ? x : xMax);
                yMin = (y < yMin ? y : yMin);
                yMax = (y > yMax ? y : yMax);
            } else {
                invertible = false;
            }
        }
    }
}

std::pair<float,float> InterpolatedFunction::xRange()
{
    return std::make_pair(xMin, xMax);
}

std::pair<float,float> InterpolatedFunction::yRange()
{
    return std::make_pair(yMin, yMax);
}


float InterpolatedFunction::getY(float x)
{
    int n = data.size();
    if (x >= xMin && x <= xMax && n >= 2) {
        for (int i=0; i<n-1; i++) {
            std::pair<float,float>  &cur = data[i];
            std::pair<float,float>  &next = data[i+1];
            if (x >= cur.first && x <= next.first) {
                float relative = (x - cur.first) / (next.first - cur.first);
                float y = cur.second + relative*(next.second - cur.second);
                return y;
            }
        }
    }

    return 0;
}

float InterpolatedFunction::getX(float y)
{
    int n = data.size();
    if (y >= yMin && y <= yMax && n >= 2) {
        for (int i=0; i<n-1; i++) {
            std::pair<float,float>  &cur = data[i];
            std::pair<float,float>  &next = data[i+1];
            if (y >= cur.second && y <= next.second) {
                float relative = (y - cur.second) / (next.second - cur.second);
                float x = cur.first + relative*(next.first - cur.first);
                return x;
            }
        }
    }

    return 0;
}






ResizeJob::ResizeJob(
        QSharedPointer<RenderedCalibrationImage> atag,
        QImage *aimage,
        QString afilename
    ) :
    tag(atag),
    image(aimage),
    filename(afilename)
{

}


ResizeJobQueue::ResizeJobQueue() :
    is_aborting(false)
{
}

void ResizeJobQueue::clear()
{
    QMutexLocker        l(&lock_jobs);
    jobs.clear();
    signal_read.wakeAll();
    signal_write.wakeAll();
}

void ResizeJobQueue::abort()
{
    is_aborting = true;
}

void ResizeJobQueue::push(QSharedPointer<ResizeJob> job)
{
    while (true) {
        if (is_aborting) return ;

        {
            QMutexLocker        l(&lock_jobs);
            if (jobs.count() < 30) {
                jobs.push_back(job);
                signal_read.wakeOne();
                return ;
            }
        }

        {
            QMutexLocker        l(&lock_write);
            signal_write.wait(&lock_write, 20);
        }
    }
}

QSharedPointer<ResizeJob> ResizeJobQueue::get()
{
    QSharedPointer<ResizeJob>   result;

    while (true) {

        {
            QMutexLocker        l(&lock_jobs);
            if (jobs.count() > 0) {
                result = jobs.front();
                jobs.pop_front();
                signal_write.wakeOne();
                return result;
            } else {

                if (is_aborting) {
                    return nullptr;
                }

            }
        }

        {
            QMutexLocker        l(&lock_read);
            // Wait 20milliseconds for something new
            signal_read.wait(&lock_read, 20);
        }
    }

    return nullptr;
}





}



