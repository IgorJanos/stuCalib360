#ifndef EXPORTER_H
#define EXPORTER_H


#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>


class CheckedImagesModel;

//-----------------------------------------------------------------------------
//
//  Exporting API
//
//-----------------------------------------------------------------------------

namespace Exporter {


/*
 *      Camera Orientation - R = Rz(pan) * Rx(tilt) * Rz(roll)
 *
 *      Vector3d for orientation has <pan,tilt,roll>
 *
 * */


class Orientation
{
public:
    double      pan;                    // default in radians
    double      tilt;
    double      roll;

public:
    Orientation();
    Orientation(double p, double t, double r);
    Orientation(cv::Vec3d o);
    Orientation(const Orientation &o);
    Orientation &operator =(const Orientation &o);

    static Orientation zxz_from_rotation(cv::Mat R);

    cv::Mat rotation_zxz();
    cv::Mat rotation_xyz();

    operator cv::Vec3d() { return cv::Vec3d(pan, tilt, roll); }
};












double fov_to_f(double fov);
double f_to_fov(double f, double x);
cv::Mat _toGL();
cv::Mat _toSoccerNet();

cv::Mat _camera(double width, double height, double fov_v_rad);

cv::Mat _rx(double a);
cv::Mat _ry(double a);
cv::Mat _rz(double a);


cv::Mat _identity();
cv::Mat _fromQ(QMatrix4x4 m);
QMatrix4x4 _toQ(cv::Mat m);
cv::Mat _translate(cv::Vec3d v);
cv::Mat _rotate_rad(cv::Vec3d a);
cv::Mat _rotate_deg(cv::Vec3d a);
cv::Mat _lookat(cv::Vec3d eye, cv::Vec3d target, cv::Vec3d up);

cv::Vec3d _transform(cv::Vec3d v, cv::Mat C);

bool isRotationMatrix(cv::Mat &R);
cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R);


cv::Mat _extract_rotation_from_4x4(cv::Mat m);
cv::Mat _mv_from_rv_tv(cv::Mat rvec, cv::Mat tvec);


typedef QList<std::pair<cv::Vec2d, bool>>     PointList;

int _find_homography(
    QSharedPointer<Data::ImageMapping> amapping,
    PointList imagepoints,
    QRect rcClip,
    // out
    cv::Mat &K,
    cv::Mat &R, cv::Mat &T
    );


cv::Mat get_panorama_inverse_V(cv::Mat V);

PointList project_panorama_to_pinhole(
    QSharedPointer<Data::ImageMapping> amapping,
    cv::Mat K, cv::Mat P,
    double imw, double imh
    );


class HomographyGT
{
public:

    cv::Vec3d       position;
    Orientation     orientation;
    double          fov_v, fov_h;

public:
    HomographyGT();
    HomographyGT(const HomographyGT &v);
    HomographyGT &operator =(HomographyGT &v);
};


class MappingCalibration
{
public:

    QSharedPointer<Data::ImageMapping>  mapping;
    cv::Size                            image_size;

    // Initial calibration    
    cv::Vec3d       mean_position;
    Orientation     mean_orientation;
    cv::Mat         B_calibration;

    // Current view
    Orientation     view_orientation;
    cv::Mat         B, V;

    // Extracted information for current view
    cv::Vec3d       base_position;
    Orientation     base_orientation;


public:
    MappingCalibration();

    void calibrate(QSharedPointer<Data::ImageMapping> amapping, cv::Size aimage_size);
    PointList project_for_view(cv::Mat K);

    void set_view(Orientation o, double fov_v_rad);
    bool optimize_view(double fov_v_rad);
    void look_at(cv::Vec3d target, double fov_v_rad);


    // Helper methods
    void mv_to_pos_orientation(
        cv::Mat mv,
        cv::Mat V,
        cv::Vec3d &p, cv::Mat &b_rot
        );

    void pos_orientation_to_mv(
        cv::Vec3d p, Orientation base_orientation,
        cv::Mat V,
        cv::Mat &mv
        );

    bool get_position_orientation(
        QSharedPointer<Data::ImageMapping> amapping,
        cv::Mat V, double fov_v_rad,
        cv::Size image_size, cv::Size view_size,
        int min_num_points,
        cv::Vec3d &position, cv::Vec3d &rvec
        );

    HomographyGT get_homography_gt(cv::Size image_size, double fov_v_rad);
};



template<class T>
class PipelineSource
{
public:

    virtual QSharedPointer<T> current() = 0;
    virtual void reset() = 0;
    virtual void next() = 0;
    virtual bool hasCurrent() = 0;

};

class Image
{
public:
    QString                                 filename;       // 001.jpg
    QImage                                  image;
    cv::Size                                imageSize;
    QSharedPointer<Data::ImageMapping>      mapping;
    QSharedPointer<MappingCalibration>      mappingCalibration;
};

class RenderedImage
{
public:
    QImage                                  image;
    QSharedPointer<Data::ImageMapping>      mapping;
    cv::Mat                                 RK_inverse;

    // Reprojected points with their visibility
    //
    //  (3 x N-points)
    //
    //  x0  x1              xN
    //  y0  y1 ... ... ...  yN
    //  v0  v1              vN
    cv::Mat                                 labelVolume;
};

class RenderedCalibrationImage
{
public:
    QImage                  imagePlayfield;
    QList<QImage>           gridImages;
    double                  fov_v, fov_h;       // in degrees
};



class DatasetImageSource : public PipelineSource<Image>
{
protected:

    QMutex              lock;

    QString             path;
    QStringList         images;
    int                 index;

    QSharedPointer<Data::PlayfieldModel> pfModel;

    QString mappingFile(QString image) const;

public:
    DatasetImageSource(DatasetModel *amodel, CheckedImagesModel *achmodel, PlayfieldCollectionModel *pfcm);

    // PipelineSource
    virtual QSharedPointer<Image> current();
    virtual void reset();
    virtual void next();
    virtual bool hasCurrent();

};


class CycleCounter : public PipelineSource<Image>
{
protected:

    QMutex              lock;

    QSharedPointer<PipelineSource<Image>>       source;
    int                                         count;
    int                                         index;

public:
    CycleCounter(QSharedPointer<PipelineSource<Image>> asrc, int acount);

    // PipelineSource
    virtual QSharedPointer<Image> current();
    virtual void reset();
    virtual void next();
    virtual bool hasCurrent();

};

class Repeater : public PipelineSource<Image>
{
protected:

    QMutex              lock;

    QSharedPointer<Image>                       cache;
    QSharedPointer<PipelineSource<Image>>       source;
    int                                         count;
    int                                         index;

public:
    Repeater(QSharedPointer<PipelineSource<Image>> asrc, int acount);

    // PipelineSource
    virtual QSharedPointer<Image> current();
    virtual void reset();
    virtual void next();
    virtual bool hasCurrent();

};

class PlayfieldGridImages
{
public:

    QList<QSharedPointer<QImage>>       images;

public:
    PlayfieldGridImages();
    PlayfieldGridImages(const PlayfieldGridImages &v);
    PlayfieldGridImages &operator =(PlayfieldGridImages v);

    static QSharedPointer<PlayfieldGridImages> FromFolder(QString path);

};

class CropArgs
{
public:

    std::pair<float, float>     rangePan;
    std::pair<float, float>     rangeTilt;
    std::pair<float, float>     rangeRoll;
    std::pair<float, float>     rangeFOV;

    // Distortion
    std::pair<float, float>     rangeK1;
    std::pair<float, float>     rangeK2;

    bool                        useDistortion;
    bool                        useColorAdjust;

public:
    CropArgs();

};

class CropSample
{
public:

    float           p, t, r;
    float           fov;
    float           k1, k2;

    float           gamma;
    float           h, s, v;

public:
    CropSample();
    CropSample(const CropSample &v);
    CropSample &operator =(CropSample v);

    static CropSample Random(const CropArgs &args);
};


class CalibrationSampleArgs
{
public:

    std::pair<double, double>                 rangeRoll;
    std::pair<double, double>                 rangeFOV;
    QSharedPointer<Data::PlayfieldModel>    playfieldModel;

public:
    CalibrationSampleArgs();

};

class CalibrationSample
{
public:

    double           roll;
    double           fov;
    cv::Vec3d        camera_target;

public:
    CalibrationSample();
    CalibrationSample(const CalibrationSample &v);
    CalibrationSample &operator =(CalibrationSample v);

    static CalibrationSample Random(const CalibrationSampleArgs &args, cv::Vec3d camera_pos);
};



class CalibrationCameraPose
{
public:

    double         x, y, z;
    double         pan, tilt, roll;        // in degrees - absolute
    double         fov_v;                  // in degrees
    Orientation    view_orientation;       // view only

    cv::Mat        mv;                     // modelview matrix

public:
    CalibrationCameraPose();
    CalibrationCameraPose(const CalibrationCameraPose &v);
    CalibrationCameraPose &operator =(CalibrationCameraPose v);

};


void computeCameraPose(
    QSharedPointer<Image> image,
    CalibrationSample &sample,
    CalibrationCameraPose &result
    );


void invertCoeffs(float *src, float *dst, int n=4);
void invertCoeffs_Eigen(float *src, float *dst, int n=4);

QVector4D getInverseK(QVector4D k);
QVector4D getInverseK(float k1, float k2);
float k2Fromk1(float k1);


class BaseExportProcessState
{
protected:

    QMutex          lock;
    bool            isAborting;
    int             totalCount;
    int             progress;

public:
    BaseExportProcessState(int atotal);

    void abort();
    void next();
    void getProgress(int &aprogress, int &atotal);

    inline bool shouldAbort() { return isAborting; }
};


class ExportProcessState :
    public BaseExportProcessState
{
protected:

    QSharedPointer<RenderedImage>  currentFrame;

public:
    ExportProcessState(int atotal);

    void setCurrentFrame(QSharedPointer<RenderedImage> frame);

    inline QSharedPointer<RenderedImage> getCurrentFrame()
    {
        QMutexLocker    l(&lock);
        return currentFrame;
    }
};

class ExportCalibrationProcessState :
    public BaseExportProcessState
{
protected:

    QSharedPointer<RenderedCalibrationImage>  currentFrame;

public:
    ExportCalibrationProcessState(int atotal);

    void setCurrentFrame(QSharedPointer<RenderedCalibrationImage> frame);

    inline QSharedPointer<RenderedCalibrationImage> getCurrentFrame()
    {
        QMutexLocker    l(&lock);
        return currentFrame;
    }
};


double toRad(double degrees);
cv::Mat RotationMatrix(double rx, double ry, double rz);



class PinholeProgram
{
private:

    QOpenGLFunctions        *f;
    bool                    heightWise;

    // Rendering program
    QOpenGLShaderProgram    *program;
    GLint                   posVertex;
    GLint                   posTex;
    GLint                   posTexture;
    GLint                   posDistortTexture;
    GLint                   posCanvas;
    GLint                   posRK;
    GLint                   posArgs;
    GLint                   posK;

    void setCanvas(QVector2D value);
    void setRK(QMatrix3x3 value);

public:
    PinholeProgram(QOpenGLFunctions *func);

    void init();
    void destroy();
    void bind();
    void unbind();

    void prepareView(CropSample s, int width, int height);
    //cv::Mat getInverseRK(CropSample s, int width, int height);

    void setTexture(GLint value);
    void setDistortTexture(GLint value);
    void setArgs(float gamma, QVector3D hsv);
    void setK(QVector4D k);

    void draw();

    inline bool HeightWise() { return heightWise; }
};

class PinholePlayfieldProgram
{
private:
    QOpenGLFunctions        *f;
    bool                    heightWise;

    // Rendering program
    QOpenGLShaderProgram    *program;
    GLint                   posVertex;
    GLint                   posTex;
    GLint                   posTexture;
    GLint                   posMVP;

    QMatrix4x4              toGL;

public:
    PinholePlayfieldProgram(QOpenGLFunctions *func);

    void init();
    void destroy();
    void bind();
    void unbind();

    void prepareView(
        QMatrix4x4 view,
        float fov_v_deg, double aspect
        );
    void setTexture(GLint value);

    void draw();
};



class InterpolatedFunction
{
protected:

    bool                                    invertible;
    std::vector<std::pair<float,float>>     data;
    float                                   xMin, xMax, yMin, yMax;

public:
    InterpolatedFunction();

    void add(float x, float y);
    void reset();

    float getY(float x);
    float getX(float y);

    std::pair<float,float> xRange();
    std::pair<float,float> yRange();
};



class RenderTarget
{
protected:
    QSize                   size;
    GLuint                  rtTarget, dsTarget;
    GLuint                  fbo;
    uchar                   *pixels;

public:
    RenderTarget(QSize asize);
    ~RenderTarget();

    void init(QOpenGLFunctions *f);
    void bind(QOpenGLFunctions *f);
    void unbind(QOpenGLFunctions *f);
    void download(QOpenGLFunctions *f, QImage &result);
};

class TextureCache
{
protected:

    struct Item {
        QSharedPointer<Image>   image;
        QSharedPointer<QImage>  qimage;
        QOpenGLTexture          *texture;
    };

    QMap<QString, Item>     items;

public:
    TextureCache();
    ~TextureCache();

    void clear();
    QOpenGLTexture *get(QString key, QSharedPointer<Image> image);
    QOpenGLTexture *get(QString key, QSharedPointer<QImage> image);
};

class DistortionTexture
{
protected:

    QSize               size;
    QOpenGLTexture      *texture;

    void computeDistortTexture(InterpolatedFunction &func, float maxR, bool inverse);

public:
    DistortionTexture(QSize asize);
    ~DistortionTexture();

    void clear();
    QOpenGLTexture *get(QVector4D k, InterpolatedFunction &dist);
};


class CropRenderer
{
protected:

    QSize                   size;
    QOpenGLContext          *context;
    QOffscreenSurface       *surface;
    PinholeProgram          *program;

    RenderTarget            rt;
    TextureCache            textures;
    DistortionTexture       texDistort;
    InterpolatedFunction    dist;


    bool isInitialized;

protected:

    bool initialize();

    cv::Mat reproject(
                    QSharedPointer<Data::ImageMapping> mapping,
                    cv::Mat rk_inv, QVector4D k, float imw, float imh
                );

public:
    CropRenderer(QOffscreenSurface *asurface, QSize asize);
    virtual ~CropRenderer();

    // Rendering
    QSharedPointer<RenderedImage> render(QSharedPointer<Image> image, CropSample sample);

};


class CalibrationRenderer
{
protected:

    QSize                   size;
    QOpenGLContext          *context;
    QOffscreenSurface       *surface;
    PinholeProgram          *program;
    PinholePlayfieldProgram *playfieldProgram;

    RenderTarget            rt;
    TextureCache            textures;
    DistortionTexture       texDistort;
    InterpolatedFunction    dist;


    bool isInitialized;

protected:

    bool initialize();

    void renderMain(
        QSharedPointer<Image> image,
        CalibrationCameraPose pose,
        QSharedPointer<RenderedCalibrationImage> result
        );
    void renderGrid(
        CalibrationCameraPose pose,
        QSharedPointer<PlayfieldGridImages> gridImages,
        QSharedPointer<RenderedCalibrationImage> result
        );

public:
    CalibrationRenderer(QOffscreenSurface *asurface, QSize asize);
    virtual ~CalibrationRenderer();

    // Rendering
    QSharedPointer<RenderedCalibrationImage> render(
        QSharedPointer<Image> image,
        QSharedPointer<PlayfieldGridImages> gridImages,
        CalibrationCameraPose pose
        );
};



class DatasetSink
{
protected:

    QString             path;
    int                 totalCount;
    cv::Size            scaleSize;

    QFile               *labelsCSV;
    int                 index;
    int                 padding;
    QString             imagePrefix;

    bool                writeLabelImages;

protected:

    QString getImageFilename(int aindex);
    QString labelToString(cv::Mat volume);
    QString getBestPrefix();

public:
    DatasetSink(QString apath, int acount, int asw, int ash, bool alabelimages);
    virtual ~DatasetSink();

    // Write
    void write(QSharedPointer<RenderedImage> frame, CropSample sample);

};




class ResizeJob
{
public:
    QSharedPointer<RenderedCalibrationImage>    tag;
    QImage                                      *image;
    QString                                     filename;

public:
    ResizeJob(
        QSharedPointer<RenderedCalibrationImage> atag,
        QImage *aimage,
        QString afilename
    );

};


class ResizeJobQueue
{
protected:

    QMutex                              lock_jobs;
    QMutex                              lock_read;
    QMutex                              lock_write;
    QWaitCondition                      signal_read;
    QWaitCondition                      signal_write;

    QList<QSharedPointer<ResizeJob>>    jobs;
    bool                                is_aborting;

public:
    ResizeJobQueue();

    void clear();
    void abort();
    void push(QSharedPointer<ResizeJob> job);
    QSharedPointer<ResizeJob> get();

    inline bool aborting() { return is_aborting; }
};


class ResizeJobWorker : public QThread
{
protected:

    cv::Size            scaleSize;
    ResizeJobQueue      *queue;

public:
    ResizeJobWorker(cv::Size asize, ResizeJobQueue *aqueue);

    void run() override;
};


class CalibrationDatasetSink
{
protected:

    QString             path;
    int                 totalCount;
    cv::Size            scaleSize;
    int                 index;

    QFile                       *labelsCSV;

    ResizeJobQueue              queue;
    QList<ResizeJobWorker*>     workers;

protected:

    QString getBasename(int aindex);

    void saveResized(QString baseName, QString suffix, QImage &image);

    QSharedPointer<ResizeJob> get_job(
            QSharedPointer<RenderedCalibrationImage> frame,
            QImage *aimage,
            QString base_name,
            QString suffix
        );

public:
    CalibrationDatasetSink(QString apath, int acount, int asw, int ash);
    virtual ~CalibrationDatasetSink();

    // Write
    void start();
    void stop();
    void write(
        QSharedPointer<RenderedCalibrationImage> frame,
        CalibrationCameraPose pose
        );

};









}

#endif // EXPORTER_H







