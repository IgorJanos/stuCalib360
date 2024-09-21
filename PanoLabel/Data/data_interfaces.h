//-----------------------------------------------------------------------------
//
//  Label Tool
//
//  Author : Igor Janos
//
//-----------------------------------------------------------------------------
#ifndef DATA_INTERFACES_H
#define DATA_INTERFACES_H


namespace Data {


    struct KeyPoint
    {
        double       x, y, z;

        KeyPoint(double ax, double ay, double az) :
            x(ax), y(ay), z(az)
        {
        }
    };

    class PlayfieldModel
    {
    public:

        QString                             filename;
        QString                             name;
        float                               length, width;
        std::vector<KeyPoint>               keyPoints;
        std::vector<std::pair<int, int>>    edges;
        bool                                is_flipped;

    public:
        PlayfieldModel();

        PlayfieldModel *clone();

        bool read(const QJsonObject &json);
        void write(QJsonObject &json) const;

        void flip_yz_coordinates();

    public:

        static PlayfieldModel *fromFile(QString adir, QString afileName);
        bool toFile(QString adir);

    };


    class PlayfieldModelGenerator
    {
    public:
        void generate(PlayfieldModel *amodel, qreal alength, qreal awidth);
    };



    class KeyPointMapping
    {
    public:
        KeyPoint                kp;         // 3D world coordinates
        double                  x, y;       // image pixel coordinates
    public:
        KeyPointMapping();
        KeyPointMapping(KeyPoint &akp, double ax, double ay);
        KeyPointMapping(const KeyPointMapping &akp);
        KeyPointMapping &operator =(const KeyPointMapping &akp);

        static cv::Vec3f vectorFrom(KeyPointMapping &kp1, KeyPointMapping &kp2);
    };


    class ImageMapping
    {
    public:
        QString                             mappingFileName;
        QString                             imageFileName;
        QList<KeyPointMapping>              kps;
        std::vector<std::pair<int, int>>    edges;

        KeyPoint                            camera_position;
        KeyPoint                            camera_orientation;
        bool                                is_position_valid;

    public:
        ImageMapping();

        bool read(const QJsonObject &json, PlayfieldModel *amodel);
        void write(QJsonObject &json) const;

        static ImageMapping *loadFromFile(QString filename, PlayfieldModel *amodel);

        void toFile();
        void loadKeyPoints(PlayfieldModel *amodel, int cx, int cy);

        static ImageMapping *tesselate(ImageMapping *am, float amaxLength, int imw, int imh);
    };

    class Dataset
    {
    public:

        QString     datasetName;
        QString     modelFilename;

    public:
        Dataset();

        static Dataset *loadFromFolder(QString path);

        bool read(const QJsonObject &json);
        void write(QJsonObject &json) const;

        void toFile(QString basepath);
    };



}


#endif // DATA_INTERFACES_H
