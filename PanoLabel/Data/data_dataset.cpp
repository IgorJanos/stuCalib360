
#include "pch.h"

#include "data_interfaces.h"


namespace Data {



    KeyPointMapping::KeyPointMapping() :
        kp(0, 0, 0),
        x(0.0),
        y(0.0)
    {
    }

    KeyPointMapping::KeyPointMapping(KeyPoint &akp, double ax, double ay) :
        kp(akp),
        x(ax),
        y(ay)
    {
    }

    KeyPointMapping::KeyPointMapping(const KeyPointMapping &akp) :
        kp(akp.kp),
        x(akp.x),
        y(akp.y)
    {
    }

    KeyPointMapping &KeyPointMapping::operator =(const KeyPointMapping &akp)
    {
        kp = akp.kp;
        x = akp.x;
        y = akp.y;
        return *this;
    }

    cv::Vec3f KeyPointMapping::vectorFrom(KeyPointMapping &kp1, KeyPointMapping &kp2)
    {
        cv::Vec3f result(
                    kp2.kp.x - kp1.kp.x,
                    kp2.kp.y - kp1.kp.y,
                    kp2.kp.z - kp1.kp.z
        );
        return result;
    }


    ImageMapping::ImageMapping() :
        camera_position(0.0f, 0.0f, 0.0f),
        camera_orientation(0.0f, 0.0f, 0.0f),
        is_position_valid(false)
    {
    }

    ImageMapping *ImageMapping::loadFromFile(QString filename, PlayfieldModel *amodel)
    {
        ImageMapping *result = nullptr;

        QFile           file(filename);
        if (file.open(QIODevice::ReadOnly)) {

            QByteArray      fileData = file.readAll();
            QJsonParseError err;
            QJsonDocument   json(QJsonDocument::fromJson(fileData, &err));

            // Skusime loadnut
            result = new ImageMapping();
            if (result->read(json.object(), amodel)) {
                result->mappingFileName = filename;
            } else {
                delete result;
                result = nullptr;
            }
        }

        return result;
    }


    bool ImageMapping::read(const QJsonObject &json, PlayfieldModel *amodel)
    {
        kps.clear();

        if (json.contains("image") && json["image"].isString()) {
            imageFileName = json["image"].toString();
        }

        if (amodel) {
            if (json.contains("kps") && json["kps"].isArray()) {
                // loadujeme obrazky
                auto items = json["kps"].toArray();
                for (int i=0; i<items.size(); i++) {
                    if (items[i].isArray()) {

                        auto item = items[i].toArray();
                        if (item.size() == 2) {

                            KeyPointMapping     kpm;

                            // Vezmeme z modelu
                            if (i < amodel->keyPoints.size()) {
                                kpm.kp = amodel->keyPoints[i];
                            }

                            // Suradnice z JSONu
                            kpm.x = item[0].toDouble();
                            kpm.y = item[1].toDouble();

                            // Odkladame si
                            kps.push_back(kpm);
                        }
                    }
                }
            }

            if (json.contains("edges") && json["edges"].isArray()) {
                // loadujeme hrany
                auto items = json["edges"].toArray();
                for (int i=0; i<items.size(); i++) {
                    if (items[i].isArray()) {
                        auto item = items[i].toArray();

                        int i0 = item[0].toInt();
                        int i1 = item[1].toInt();

                        // pridavame
                        edges.push_back(std::pair<int, int>(i0, i1));
                    }

                }
            }
        }

        is_position_valid = false;
        if (json.contains("camera_position") &&
            json["camera_position"].isArray()
            ) {
            auto item = json["camera_position"].toArray();
            camera_position.x = item[0].toDouble();
            camera_position.y = item[1].toDouble();
            camera_position.z = item[2].toDouble();
            is_position_valid = true;
        }

        if (json.contains("camera_orientation") &&
            json["camera_orientation"].isArray()
            ) {
            auto item = json["camera_orientation"].toArray();
            camera_orientation.x = item[0].toDouble();
            camera_orientation.y = item[1].toDouble();
            camera_orientation.z = item[2].toDouble();
        }

        return true;
    }

    void ImageMapping::write(QJsonObject &json) const
    {
        json["image"] = imageFileName;

        QJsonArray items;
        for (int i=0; i<(int)kps.count(); i++) {
            QJsonArray item;
            item.push_back(kps[i].x);
            item.push_back(kps[i].y);
            items.push_back(item);
        }

        QJsonArray     aedges;
        for (int i=0; i<edges.size(); i++) {
            QJsonArray aedge;
            aedge.append(edges[i].first);
            aedge.append(edges[i].second);

            // pridame novy edge
            aedges.append(aedge);
        }

        json["kps"] = items;
        json["edges"] = aedges;

        if (is_position_valid) {
            QJsonArray      pos;
            pos.append(camera_position.x);
            pos.append(camera_position.y);
            pos.append(camera_position.z);

            QJsonArray      orientation;
            orientation.append(camera_orientation.x);
            orientation.append(camera_orientation.y);
            orientation.append(camera_orientation.z);

            json["camera_position"] = pos;
            json["camera_orientation"] = orientation;
        }
    }

    void ImageMapping::toFile()
    {
        QFile       file(mappingFileName);
        if (file.open(QIODevice::WriteOnly)) {

            QJsonObject     json;
            write(json);

            // sup ho do suboru
            file.write(QJsonDocument(json).toJson());
        }
    }

    void ImageMapping::loadKeyPoints(PlayfieldModel *amodel, int cx, int cy)
    {
        // Skopirujeme vsetky keypointy
        kps.clear();
        edges.clear();

        double scaleFactor = 10.0;

        for (int i=0; i<(int)amodel->keyPoints.size(); i++) {

            KeyPointMapping kpm;

            kpm.kp = amodel->keyPoints[i];
            kpm.x = cx + kpm.kp.x * scaleFactor;
            kpm.y = cy + kpm.kp.z * scaleFactor;

            // Prihodime novy key point
            kps.push_back(kpm);
        }

        // Skopirujeme aj zoznam hran
        edges = amodel->edges;
    }


    ImageMapping *ImageMapping::tesselate(ImageMapping *am, float amaxLength, int imw, int imh)
    {
        ImageMapping *result = new ImageMapping();

        //  Najskor skopirujeme vsekty povodne kps
        for (int i=0; i<(int)am->kps.size(); i++) {
            result->kps.push_back(am->kps[i]);
        }

        //  Prebehneme vsetky segmenty a rozbijeme ich na mensie
        for (int i=0; i<(int)am->edges.size(); i++) {
            auto &edge = am->edges[i];
            auto &kpStart = result->kps[edge.first];
            auto &kpEnd = result->kps[edge.second];

            cv::Vec3f v = KeyPointMapping::vectorFrom(kpStart, kpEnd);
            double distance = cv::norm(v);

            if (distance > amaxLength) {

                int nSegments = ceil(distance / amaxLength);
                cv::Vec3f unit = v / nSegments;

                // Budeme mat (nSegments-1) novych keypointov
                int iStart = result->kps.size();
                for (int n=0; n<(nSegments-1); n++) {
                    KeyPointMapping     kp;
                    kp.kp.x = kpStart.kp.x + (n+1)*unit(0);
                    kp.kp.y = kpStart.kp.y + (n+1)*unit(1);
                    kp.kp.z = kpStart.kp.z + (n+1)*unit(2);

                    result->kps.push_back(kp);
                }

                // Budeme mat (nSegments) novych edgov
                result->edges.push_back(std::make_pair(edge.first, iStart));
                int iCur = iStart;
                for (int n=0; n<(nSegments-2); n++) {
                    result->edges.push_back(std::make_pair(iCur, iCur+1));
                    iCur = iCur+1;
                }
                result->edges.push_back(std::make_pair(iCur, edge.second));

            } else {
                // Pridame tak ako toje
                result->edges.push_back(edge);
            }

        }

        return result;
    }





    //-------------------------------------------------------------------------
    //  Dataset class
    //-------------------------------------------------------------------------

    Dataset::Dataset()
    {

    }

    Dataset *Dataset::loadFromFolder(QString path)
    {
        Dataset *result = new Dataset();

        QDir            dir(path);
        result->datasetName = dir.dirName();

        QFile           file(path + "/dataset.json");
        if (file.open(QIODevice::ReadOnly)) {

            QByteArray      fileData = file.readAll();
            QJsonParseError err;
            QJsonDocument   json(QJsonDocument::fromJson(fileData, &err));

            // Skusime loadnut
            if (result->read(json.object())) {
            } else {
                delete result;
                result = nullptr;
            }
        }

        return result;
    }


    bool Dataset::read(const QJsonObject &json)
    {
        if (json.contains("model") && json["model"].isString()) {
            modelFilename = json["model"].toString();
        }

        return true;
    }

    void Dataset::write(QJsonObject &json) const
    {
        json["model"] = modelFilename;
    }

    void Dataset::toFile(QString basepath)
    {
        QDir        dir(basepath + "/" + datasetName);
        dir.mkpath("images");
        dir.mkpath("mapping");

        QFile       file(dir.absolutePath() + "/dataset.json");
        if (file.open(QIODevice::WriteOnly)) {

            QJsonObject     json;
            write(json);

            // sup ho do suboru
            file.write(QJsonDocument(json).toJson());
        }
    }


}
