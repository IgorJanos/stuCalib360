#include "pch.h"
#include <math.h>

using namespace std;
using namespace cv;


float to_degrees(float v)
{
    return v*180.0 / M_PI;
}





//-----------------------------------------------------------------------------
//
//  PTZOperation class
//
//-----------------------------------------------------------------------------

PTZOperation::PTZOperation(
        PinholePreviewer *aview, QMouseEvent *aevent
        ) :
    view(aview),
    pos(aevent->pos())
{
    // Povodne data
    vdStart = view->view;
}

void PTZOperation::mouseMoveEvent(QMouseEvent *event)
{
    // Rozdiel oproti povodnemu stavu
    QPoint      delta = event->pos() - pos;

    // O kolko stupnov sa mame pootocit ?
    float       degPX = vdStart.fov / view->height();

    // Aktualizujeme novu direction
    view->view.p = vdStart.p - delta.x() * degPX;
    view->view.t = vdStart.t + delta.y() * degPX;

    view->updateViewMatrix();
    view->update();
}


PinholeViewPort g_PinholeViewport;


PinholeViewPort::PinholeViewPort()
{
    // default ranges
    rangeK1 = std::make_pair(-0.7, 0.3);
    rangeK2 = std::make_pair(-0.2, 0.2);

    labelEnabled = false; //true;
    finalEnabled = false; //true;
    gridEnabled = true;
    k = QVector4D(0.0, 0.0, 0.0, 0.0);
}

PinholeViewPort &PinholeViewPort::get() { return g_PinholeViewport; }

//-----------------------------------------------------------------------------
//
//  PinholePreviewer class
//
//-----------------------------------------------------------------------------

PinholePreviewer::PinholePreviewer(QWidget *parent) :
    QOpenGLWidget(parent),
    texture_grid(nullptr),
    texture(nullptr),
    texDistort(nullptr)
{
    setFocusPolicy(Qt::StrongFocus);
    program = new Exporter::PinholeProgram(this);
    playfieldProgram = new Exporter::PinholePlayfieldProgram(this);

    // New calibration
    mappingCalibration = QSharedPointer<Exporter::MappingCalibration>(
        new Exporter::MappingCalibration()
        );
}

PinholePreviewer::~PinholePreviewer()
{
    cleanup();
}


void PinholePreviewer::cleanup()
{
    makeCurrent();

    if (texture) {
        delete texture;
        texture = nullptr;
    }

    if (texture_grid) {
        delete texture_grid;
        texture_grid = nullptr;
    }

    if (texDistort) {
        delete texDistort;
        texDistort = nullptr;
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

    doneCurrent();
}



//-----------------------------------------------------------------------------
//  QOpenGLWidget
//-----------------------------------------------------------------------------


static float distort(QVector4D k, float r)
{
    float r2 = r*r;
    float r4 = r2*r2;
    double d = 1.0 + k.x()*r2 + k.y()*r4 + k.z()*(r2*r4) + k.w()*(r4*r4);
    return d;
}

static std::pair<cv::Vec2d,bool> distort(cv::Vec2d p, cv::Vec2d c, float *k, float h, float maxR)
{
    double x = (double)(p[0] - c[0]) / (double)h;
    double y = (double)(p[1] - c[1]) / (double)h;
    double r2 = (x*x + y*y);
    double r4 = r2*r2;
    double r8 = r4*r4;

    // distort ratio
    double d = 1.0
            + k[0]*r2         + k[1]*r4
            + k[2]*(r2*r4)    + k[3]*(r8)
            ;

    double _x = c[0] + x*d*h;
    double _y = c[1] + y*d*h;


    double r = sqrt(r2);
    bool valid = (r <= maxR);

    return std::make_pair(cv::Vec2d(_x, _y), valid);
}




static Exporter::PointList distortPoints(
            Exporter::PointList points,
            cv::Mat K,
            QVector4D ak,
            float h,
            float maxR
        )
{
    Exporter::PointList   result;
    cv::Vec2d             c(K.at<double>(0, 2), K.at<double>(1,2));


    // Najskor tam a potom nazad
    float       k[] = { ak.x(), ak.y(), ak.z(), ak.w() };
    for (int i=0; i<(int)points.size(); i++) {
        auto dpt = distort(
            points[i].first, c, k, h, maxR
        );
        result.push_back(dpt); //std::make_pair(pt, points[i].second));
    }

    return result;
}


static QString position_to_string(cv::Vec3d p)
{
    return QString().asprintf("%5.4f, %5.4f, %5.4f", p[0], p[1], p[2]);
}


static QString angles_to_string(cv::Vec3d a)
{
    return QString().asprintf(
        "%5.4f, %5.4f, %5.4f",
        to_degrees(a[0]),
        to_degrees(a[1]),
        to_degrees(a[2])
        );
}



static Exporter::PointList project3D(
            QSharedPointer<Data::ImageMapping> amapping,
            cv::Mat K, cv::Mat P
        )
{
    Exporter::PointList    result;

    if (amapping) {

        cv::Mat     r3d, pi;
        double      x, y, z;

        for (int i=0; i<(int)amapping->kps.size(); i++) {
            auto kp = amapping->kps[i];

            r3d = (cv::Mat_<double>(4,1) <<
                   kp.kp.x,
                   kp.kp.y,
                   kp.kp.z,
                   1.0
            );

            // Reproject into camera space
            pi = P * r3d;
            x = pi.at<double>(0, 0) / pi.at<double>(2, 0);
            y = pi.at<double>(1, 0) / pi.at<double>(2, 0);
            z = pi.at<double>(2, 0);

            // Into camera space
            x = x*K.at<double>(0,0) + K.at<double>(0,2);
            y = y*K.at<double>(1,1) + K.at<double>(1,2);

            // mapujeme jeho x, y
            cv::Vec2d p(x,y);
            result.push_back(std::make_pair(p, z>0));
        }
    }

    return result;
}

static void drawPoints(
                QPainter &p,
                QSharedPointer<Data::ImageMapping> amapping,
                Exporter::PointList points,
                QColor c1, QColor c2
                )
{
    QBrush  b1(c1);
    QBrush  b2(c2);
    QPen    p1(c1, 3.0);
    QPen    p2(c2, 1.0);

    if (amapping) {
        if (amapping->kps.size() == points.size()) {
            for (int i=0; i<(int)amapping->edges.size(); i++) {
                auto &edge = amapping->edges[i];
                auto pFrom = points[edge.first];
                auto pTo = points[edge.second];
                if (pFrom.second && pTo.second) {
                    // nakreslime

                    p.setPen(p1);
                    p.setBrush(b1);
                    p.drawEllipse(QPointF(pFrom.first[0], pFrom.first[1]), 3.0, 3.0);
                    p.drawEllipse(QPointF(pTo.first[0], pTo.first[1]), 3.0, 3.0);
                    p.drawLine(
                        QPoint(pFrom.first[0], pFrom.first[1]),
                        QPoint(pTo.first[0], pTo.first[1])
                        );

                    p.setPen(p2);
                    p.setBrush(b2);
                    p.drawEllipse(QPointF(pFrom.first[0], pFrom.first[1]), 2.0, 2.0);
                    p.drawEllipse(QPointF(pTo.first[0], pTo.first[1]), 2.0, 2.0);
                    p.drawLine(
                        QPoint(pFrom.first[0], pFrom.first[1]),
                        QPoint(pTo.first[0], pTo.first[1])
                        );

                }
            }

            // Este speci
            int idx[3] = { 8, 12, 16 };
            for (int i=0; i<3; i++) {
                auto pt = points[idx[i]];

                p.setPen(p1);
                p.setBrush(b1);
                p.drawEllipse(QPointF(pt.first[0], pt.first[1]), 3.0, 3.0);

                p.setPen(p2);
                p.setBrush(b2);
                p.drawEllipse(QPointF(pt.first[0], pt.first[1]), 2.0, 2.0);
            }
        }
    }
}



void PinholePreviewer::setMapping(QImage &aimg, QSharedPointer<Data::ImageMapping> amapping)
{
    // odlozime si velkost textury
    imageSize = cv::Size(aimg.width(), aimg.height());

    mapping = amapping;

    // reset view
    view.t = 90;
    view.p = 0;
    view.r = 0;

    // Nasekame si model na jemnejsie
    if (amapping) {
        mappingTesselated = QSharedPointer<Data::ImageMapping>(
                    Data::ImageMapping::tesselate(
                        amapping.data(), 4.0, imageSize.width, imageSize.height
                        )
                    );
    } else {
        mappingTesselated = nullptr;
    }

    // Calibrate and get the initial Base matrix
    mappingCalibration->calibrate(mapping, imageSize);

    if (texture) {
        delete texture;
        texture = nullptr;
    }

    texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
    texture->setData(aimg, QOpenGLTexture::DontGenerateMipMaps);
    texture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
    texture->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
    texture->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);

    if (!texture_grid) {
        //QString     image_path = QDir::homePath() + "/.Playfield/playfield_grid.png";
        QString     image_path = QDir::homePath() + "/.Playfield/playingfield.png";
        QImage      img(image_path);

        texture_grid = new QOpenGLTexture(QOpenGLTexture::Target2D);
        texture_grid->setData(img, QOpenGLTexture::DontGenerateMipMaps);
        texture_grid->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
        texture_grid->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
        texture_grid->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);
    }


    updateViewMatrix();
    update();
}

void PinholePreviewer::computeDistortTexture(Exporter::InterpolatedFunction func, float maxR, bool inverse)
{
    // Vypocitame maximalny diagonalny radius
    int n = 256;

    if (!texDistort) {
        texDistort = new QOpenGLTexture(QOpenGLTexture::Target1D);
        texDistort->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);

        texDistort->setSize(n);
        texDistort->setFormat(QOpenGLTexture::R32F);
        texDistort->allocateStorage(QOpenGLTexture::Red, QOpenGLTexture::Float32);
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

    texDistort->setData(QOpenGLTexture::Red, QOpenGLTexture::Float32, f.data());
}

void PinholePreviewer::initializeGL()
{
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &PinholePreviewer::cleanup);
    initializeOpenGLFunctions();

    auto f = context()->functions();

    // Setup program
    program->init();
    playfieldProgram->init();

    f->glClearColor(0.1, 0.1, 0.1, 1);

}


const float VIEW_MIN  = 5.0;
const float VIEW_MAX  = 90;


float getMaxR(int width, int height)
{
    float aspect = (float)width / (float)height;
    float h = 0.5;
    float w = h * aspect;
    return sqrt(h*h + w*w);
}



void PinholePreviewer::renderPanorama()
{
    //-------------------------------------------
    // View parameters

    PinholeViewPort &vp = PinholeViewPort::get();
    QVector4D       k = vp.k;
    QVector4D       ik = Exporter::getInverseK(k);

    // spocitame distortion funkciu
    float maxR = getMaxR(width(), height());
    int n = 256;
    auto &_dist = dist;
    _dist.reset();
    for (int i=0; i<n; i++) {
        float r = 0.0 + 2.0*maxR*(float)i/(float)(n-1);
        float d = r * distort(k, r);
        _dist.add(r, d);
    }

    computeDistortTexture(_dist, maxR, true);
    float distMaxR = _dist.xRange().second;

    //-------------------------------------------
    // Nakreslime pinhole pohlad na scenu

    auto f = context()->functions();
    f->glClear(GL_COLOR_BUFFER_BIT);

    program->bind();
    if (texture) {
        texture->bind(0);
        program->setTexture(0);
    }
    if (texDistort) {
        texDistort->bind(2);
        program->setDistortTexture(2);
    }

    program->prepareView(view, width(), height());
    program->setArgs(1.0, QVector3D(0.0, 1.0, 1.0));
    program->setK(ik);
    program->draw();
    program->unbind();

    //-------------------------------------------
    //  Nakreslenie modelu


    auto mp = this->mapping;
    auto mt = this->mappingTesselated;
    auto mc = this->mappingCalibration;
    if (mp) {


        cv::Mat     K       = Exporter::_camera(width(), height(), view.fov * M_PI/180.0);
        auto        points  = mc->project_for_view(K);

        // Este skusime spocitat homografiu
        cv::Mat     rvec, tvec;
        QRect       rcClip(0, 0, width(), height());

        // distortujeme
        // Initial estimate of Kpxk
        points = distortPoints(points, K, k, height(), distMaxR);

        // Bielou kreslime original
        QPainter    p(this);
        p.setRenderHint(QPainter::Antialiasing);

        if (vp.labelEnabled) {
            p.setPen(QColor::fromRgbF(1.0, 1.0, 1.0, 1.0));
            drawPoints(p, mp, points,
                Qt::black,
                Qt::white
            );
        }

        // Cervenou kreslime homografiu
        auto homo_gt = mc->get_homography_gt(
                cv::Size(width(), height()),
                view.fov * M_PI/180.0
            );

        if (vp.finalEnabled) {
            // Construct from HomoGT
            cv::Mat pK = Exporter::_camera(width(), height(), homo_gt.fov_v*M_PI/180.0);
            cv::Mat R = homo_gt.orientation.rotation_zxz();
            cv::Mat T = Exporter::_translate(-homo_gt.position);
            cv::Mat P = R*T;

            auto points3D = project3D(mt, pK, P);
            p.setPen(QColor::fromRgbF(1.0, 0.0, 0.0, 1.0));
            drawPoints(p, mt, points3D,
                Qt::black,
                Qt::red
            );
        }

        // Paint parameters
        if (true) {
            QFontMetrics    fm(p.font());
            QStringList     items;

            items.append("XYZ: " + position_to_string(homo_gt.position));
            items.append("PTR: " + angles_to_string((cv::Vec3d)homo_gt.orientation));
            items.append("FV,FH: " + QString().asprintf("%5.4f   %5.4f", homo_gt.fov_v, homo_gt.fov_h)
                         );


            int             tb_margin = 8;
            int             tb_left = 16;
            int             tb_top = 16;
            int             tb_width = 0;
            int             tb_height = 0;
            for (int i=0; i<items.size(); i++) {
                auto rc = fm.boundingRect(items[i]);
                tb_height += (rc.height() + tb_margin);
                tb_width = max(tb_width, rc.width());
            }
            QRect           rcBack;
            rcBack.setRect(tb_left, tb_top, tb_width + 2*tb_margin, tb_height + 2*tb_margin);
            p.setBrush(QBrush(QColor(0, 0, 0, 160)));
            p.setPen(QColor::fromRgb(0, 0, 0, 160));
            p.drawRect(rcBack);

            p.setPen(QColor::fromRgbF(1.0, 1.0, 1.0, 1.0));
            int             x = rcBack.left() + tb_margin;
            int             y = rcBack.top() + tb_margin;
            for (int i=0; i<items.size(); i++) {
                y += fm.height();
                p.drawText(x, y, items[i]);
                y += tb_margin;
            }
        }
    }
}



void PinholePreviewer::renderPlayfield()
{
    if (!mapping) return ;

    //-------------------------------------------
    // View parameters

    PinholeViewPort &vp = PinholeViewPort::get();

    //-------------------------------------------
    // Nakreslime scenu

    auto m = mapping;
    if (m->is_position_valid && vp.gridEnabled) {

        // Current view matrix is the combination of Base and View
        cv::Mat     mv = mappingCalibration->V * mappingCalibration->B;

        playfieldProgram->bind();

        if (texture_grid) {
            texture_grid->bind(0);
            playfieldProgram->setTexture(0);
        }

        playfieldProgram->prepareView(
                Exporter::_toQ(mv),
                view.fov,
                (double)width() / (double)height()
            );

        playfieldProgram->draw();
        playfieldProgram->unbind();


    }
}


void PinholePreviewer::updateViewMatrix()
{
    // Find from panorama
    if (mapping && mappingCalibration) {

        Exporter::Orientation     o_view(
            cv::Vec3d(view.p, view.t, view.r) * M_PI/180.0
        );

        mappingCalibration->set_view(o_view, view.fov * M_PI/180.0);
    }
}


void PinholePreviewer::lookAt(cv::Vec3d target)
{
    // Zmenime view pan a tilt !
    auto m = mapping;
    auto mc = mappingCalibration;
    if (m && mc) {
        mappingCalibration->look_at(target, view.fov * M_PI/180.0);

        // Load the view parameters
        view.p = mc->view_orientation.pan * 180.0/M_PI;
        view.t = mc->view_orientation.tilt * 180.0/M_PI;
        view.r = mc->view_orientation.roll * 180.0/M_PI;
    }
    update();
}


void PinholePreviewer::paintGL()
{
    renderPanorama();
    renderPlayfield();
}

void PinholePreviewer::resizeGL(int width, int height)
{
    Q_UNUSED(width);
    Q_UNUSED(height);
}


void PinholePreviewer::onViewChanged()
{
    PinholeViewPort     &vp = PinholeViewPort::get();

    // updatneme hodnoty K
    view.k1 = vp.k.x();
    view.k2 = vp.k.y();

    updateViewMatrix();
    update();
}

//-----------------------------------------------------------------------------
//  Mouse handling
//-----------------------------------------------------------------------------

void PinholePreviewer::wheelEvent(QWheelEvent *event)
{
    QPoint numPixels = event->pixelDelta();
    QPoint numDegrees = event->angleDelta() / 8;
    qreal sf = 1.0;

    if (!numPixels.isNull()) {
        sf = pow((double)2, -numPixels.y() / 240.0);
    } else if (!numDegrees.isNull()) {
        sf = pow((double)2, numDegrees.y() / 50.0);
    }

    // inverse direction
    sf = 1.0 / sf;


    view.fov *= sf;
    if (view.fov > VIEW_MAX) { view.fov = VIEW_MAX; }
    if (view.fov < VIEW_MIN) { view.fov = VIEW_MIN; }

    event->accept();

    updateViewMatrix();
    update();
}

void PinholePreviewer::mousePressEvent(QMouseEvent *event)
{
    // Zaciname daco robit
    if (event->button() == Qt::MouseButton::LeftButton) {

        if (!operation) {
            // Nova drag operacia ?
            operation = QSharedPointer<PreviewerMouseOperation>(new PTZOperation(this, event));
        }

    }
}

void PinholePreviewer::mouseReleaseEvent(QMouseEvent *event)
{
    if (operation) {
        operation->finish(event);
        operation.clear();
    }
}

void PinholePreviewer::mouseMoveEvent(QMouseEvent *event)
{
    if (operation) {
        operation->mouseMoveEvent(event);
    } else {
    }
}

//-----------------------------------------------------------------------------
//  Keyboard handling
//-----------------------------------------------------------------------------

#define HALF_LENGTH     (105.0f * 0.5f)
#define HALF_WIDTH      (68.0f * 0.5f)

void PinholePreviewer::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_R) {
        view.p = 0;
        view.t = 90;
        view.r = 0;
        updateViewMatrix();
        update();
    }
    if (event->key() == Qt::Key_A) {
        view.p -= 10.0;
        updateViewMatrix();
        update();
    }
    if (event->key() == Qt::Key_D) {
        view.p += 10.0;
        updateViewMatrix();
        update();
    }
    if (event->key() == Qt::Key_Q) {
        view.t += 10.0;
        updateViewMatrix();
        update();
    }
    if (event->key() == Qt::Key_Z) {
        view.t -= 10.0;
        updateViewMatrix();
        update();
    }
    if (event->key() == Qt::Key_X) {
        view.r -= 1.0;
        updateViewMatrix();
        update();
    }
    if (event->key() == Qt::Key_C) {
        view.r += 1.0;
        updateViewMatrix();
        update();
    }
    if (event->key() == Qt::Key_0) {
        lookAt(cv::Vec3d(0.0f, 0.0f, 0.0f));
    }
    if (event->key() == Qt::Key_1) {
        lookAt(cv::Vec3d(-HALF_LENGTH, -HALF_WIDTH, 0.0f));
    }
    if (event->key() == Qt::Key_2) {
        lookAt(cv::Vec3d(HALF_LENGTH, -HALF_WIDTH, 0.0f));
    }
    if (event->key() == Qt::Key_3) {
        lookAt(cv::Vec3d(-HALF_LENGTH, HALF_WIDTH, 0.0f));
    }
    if (event->key() == Qt::Key_4) {
        lookAt(cv::Vec3d(HALF_LENGTH, HALF_WIDTH, 0.0f));
    }
}


void PinholePreviewer::resizeEvent(QResizeEvent *event)
{
    QOpenGLWidget::resizeEvent(event);

    updateViewMatrix();
}

