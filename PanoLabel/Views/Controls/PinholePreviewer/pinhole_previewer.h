#ifndef PINHOLE_PREVIEWER_H
#define PINHOLE_PREVIEWER_H


//-----------------------------------------------------------------------------
//
//  PinholePreviewer class
//
//-----------------------------------------------------------------------------

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>

#include <QGenericMatrix>
#include <QMatrix4x4>
#include <QMatrix3x3>



class PinholePreviewer;


class PTZOperation : public PreviewerMouseOperation
{
private:
    PinholePreviewer      *view;
    QPoint                pos;
    Exporter::CropSample  vdStart;

public:
    PTZOperation(PinholePreviewer *aview, QMouseEvent *aevent);
    virtual void mouseMoveEvent(QMouseEvent *event);
};




class PinholeViewPort
{
public:

    std::pair<float, float>     rangeK1, rangeK2;

    bool            labelEnabled;
    bool            finalEnabled;
    bool            gridEnabled;
    QVector4D       k;

    PinholeViewPort();
    static PinholeViewPort &get();

};


struct CameraExtrinsics {
    cv::Vec3f   position;
    cv::Vec3f   orientation;

    CameraExtrinsics(cv::Vec3f &p, cv::Vec3f &o):
        position(p),
        orientation(o)
    {

    }
};



class PinholePreviewer :
        public QOpenGLWidget,
        public QOpenGLFunctions
{    
    friend class PTZOperation;

protected:

    QSharedPointer<PreviewerMouseOperation>     operation;
    Exporter::CropSample                        view;
    QSharedPointer<Data::ImageMapping>          mapping;
    QSharedPointer<Data::ImageMapping>          mappingTesselated;
    QSharedPointer<Exporter::MappingCalibration>      mappingCalibration;

    QOpenGLTexture                      *texture_grid;
    QOpenGLTexture                      *texture;
    Exporter::PinholeProgram            *program;
    Exporter::PinholePlayfieldProgram   *playfieldProgram;
    cv::Size                            imageSize;

    Exporter::InterpolatedFunction      dist;
    QOpenGLTexture                      *texDistort;


    // QOpenGLWidget
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;

    void computeDistortTexture(Exporter::InterpolatedFunction func, float maxR, bool inverse);

    void renderPanorama();
    void renderPlayfield();

    void lookAt(cv::Vec3d target);

    void updateViewMatrix();

public:
    PinholePreviewer(QWidget *parent = nullptr);
    ~PinholePreviewer();

    void cleanup();
    void setMapping(QImage &aimg, QSharedPointer<Data::ImageMapping> amapping);

    void resizeEvent(QResizeEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;

public slots:

    void onViewChanged();

};


#endif // PINHOLE_PREVIEWER_H
