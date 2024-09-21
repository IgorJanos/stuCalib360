
#include "pch.h"

#include "data_interfaces.h"


namespace Data {

    //-------------------------------------------------------------------------
    //
    //  PlayfieldModel class
    //
    //-------------------------------------------------------------------------

    /*
            {
                "name": "Some Name",
                "points": [
                    [0, 0, 100, 100, 0],
                    [0, 0, 100, 100, 0],
                    [0, 0, 100, 100, 0],
                    [0, 0, 100, 100, 0],
                    [0, 0, 100, 100, 0],
                    [0, 0, 100, 100, 0],
                    [0, 0, 100, 100, 0],
                ],
                "edges": [
                    [0, 1],
                    [1, 2],
                    [3, 4]
                ]
            }
    */

    PlayfieldModel::PlayfieldModel() :
        length(0), width(0),
        is_flipped(false)
    {
        // Defaults
        length = 105;
        width = 68;
    }

    PlayfieldModel *PlayfieldModel::clone()
    {
        PlayfieldModel *result = new PlayfieldModel();
        result->filename = this->filename;
        result->name = this->name;
        result->length = this->length;
        result->width = this->width;
        result->keyPoints = this->keyPoints;
        result->edges = this->edges;
        result->is_flipped = this->is_flipped;
        return result;
    }


    bool PlayfieldModel::read(const QJsonObject &json)
    {
        keyPoints.clear();
        edges.clear();

        if (json.contains("name") && json["name"].isString()) {
            name = json["name"].toString();
        }

        if (json.contains("length") && json["length"].isDouble()) {
            length = json["length"].toDouble();
        }

        if (json.contains("width") && json["width"].isDouble()) {
            width = json["width"].toDouble();
        }

        if (json.contains("points") && json["points"].isArray()) {
            // loadujeme pointy
            auto items = json["points"].toArray();
            for (int i=0; i<items.size(); i++) {
                if (items[i].isArray()) {
                    auto item = items[i].toArray();
                    float x, y, z;
                    x = item[0].toDouble();
                    y = item[1].toDouble();
                    z = item[2].toDouble();

                    // pridavame
                    keyPoints.push_back(KeyPoint(x, y, z));
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

        return true;
    }

    void PlayfieldModel::write(QJsonObject &json) const
    {                
        QJsonArray     akeyPoints;
        for (int i=0; i<keyPoints.size(); i++) {
            QJsonArray apoint;

            if (is_flipped) {
                apoint.append(keyPoints[i].x);
                apoint.append(keyPoints[i].z);
                apoint.append(keyPoints[i].y);
            } else {
                apoint.append(keyPoints[i].x);
                apoint.append(keyPoints[i].y);
                apoint.append(keyPoints[i].z);
            }


            // pridame novy point
            akeyPoints.append(apoint);
        }

        QJsonArray     aedges;
        for (int i=0; i<edges.size(); i++) {
            QJsonArray aedge;
            aedge.append(edges[i].first);
            aedge.append(edges[i].second);

            // pridame novy edge
            aedges.append(aedge);
        }

        // Ulozime to tam
        json["name"] = name;
        json["length"] = length;
        json["width"] = width;
        json["points"] = akeyPoints;
        json["edges"] = aedges;
    }

    PlayfieldModel *PlayfieldModel::fromFile(QString adir, QString afileName)
    {
        PlayfieldModel  *result = nullptr;
        QFile           file(adir + "/" + afileName);
        if (file.open(QIODevice::ReadOnly)) {

            QByteArray      fileData = file.readAll();
            QJsonParseError err;
            QJsonDocument   json(QJsonDocument::fromJson(fileData, &err));

            // Skusime loadnut
            result = new PlayfieldModel();
            if (result->read(json.object())) {
                result->filename = afileName;
                result->flip_yz_coordinates();
            } else {
                delete result;
                result = nullptr;
            }
        }

        return result;
    }

    bool PlayfieldModel::toFile(QString adir)
    {
        QFile       file(adir + "/" + filename);
        if (file.open(QIODevice::WriteOnly)) {

            QJsonObject     json;
            write(json);

            // sup ho do suboru
            file.write(QJsonDocument(json).toJson());
        }
        return true;
    }

    void PlayfieldModel::flip_yz_coordinates()
    {
        // Flip the flag!
        this->is_flipped = !this->is_flipped;

        for (int i=0; i<keyPoints.size(); i++) {
            float temp = keyPoints[i].z;
            keyPoints[i].z = keyPoints[i].y;
            keyPoints[i].y = temp;
        }
    }


    //-------------------------------------------------------------------------
    //
    //  PlayfieldModelGenerator class
    //
    //-------------------------------------------------------------------------

    void PlayfieldModelGenerator::generate(
            PlayfieldModel *amodel, qreal alength, qreal awidth
            )
    {
        auto &kp = amodel->keyPoints;
        auto &ed = amodel->edges;

        qreal       l5 = 5.5;
        qreal       l16  = 16.5;
        qreal       w16 = 40.3;
        qreal       w5 = w16 - 2*11.0;

        // Suradnice na X osi
        qreal       x0 = -alength / 2.0;
        qreal       x1 = x0 + l5;
        qreal       x2 = x0 + 11.0;
        qreal       x3 = x0 + l16;
        qreal       x4 = 0;
        qreal       x5 = -x3;
        qreal       x6 = -x2;
        qreal       x7 = -x1;
        qreal       x8 = -x0;

        // Suradnice na Y osi
        qreal       y0 = awidth / 2.0;
        qreal       y1 = w16 / 2.0;
        qreal       y2 = w5 / 2.0;
        qreal       y3 = 0.0;
        qreal       y4 = -y2;
        qreal       y5 = -y1;
        qreal       y6 = -y0;

        // Left Penalty area
        kp.push_back(KeyPoint(x0, 0, y0));
        kp.push_back(KeyPoint(x0, 0, y1));
        kp.push_back(KeyPoint(x0, 0, y2));
        kp.push_back(KeyPoint(x0, 0, y4));
        kp.push_back(KeyPoint(x0, 0, y5));
        kp.push_back(KeyPoint(x0, 0, y6));

        kp.push_back(KeyPoint(x1, 0, y2));      // hranice 5-ky
        kp.push_back(KeyPoint(x1, 0, y4));
        kp.push_back(KeyPoint(x2, 0, y3));      // Penalty spot
        kp.push_back(KeyPoint(x3, 0, y1));      // hranice 16-ky
        kp.push_back(KeyPoint(x3, 0, y5));

        // Center
        kp.push_back(KeyPoint(x4, 0, y0));
        kp.push_back(KeyPoint(x4, 0, y3));      // Center spot
        kp.push_back(KeyPoint(x4, 0, y6));

        // Right Penalty area
        kp.push_back(KeyPoint(x5, 0, y1));      // hranice 16-ky
        kp.push_back(KeyPoint(x5, 0, y5));
        kp.push_back(KeyPoint(x6, 0, y3));      // Penalty spot
        kp.push_back(KeyPoint(x7, 0, y2));      // hranice 5-ky
        kp.push_back(KeyPoint(x7, 0, y4));

        kp.push_back(KeyPoint(x8, 0, y0));
        kp.push_back(KeyPoint(x8, 0, y1));
        kp.push_back(KeyPoint(x8, 0, y2));
        kp.push_back(KeyPoint(x8, 0, y4));
        kp.push_back(KeyPoint(x8, 0, y5));
        kp.push_back(KeyPoint(x8, 0, y6));

        //---------------------------
        ed.push_back(std::pair<int,int>(0, 1));
        ed.push_back(std::pair<int,int>(1, 2));
        ed.push_back(std::pair<int,int>(2, 3));
        ed.push_back(std::pair<int,int>(3, 4));
        ed.push_back(std::pair<int,int>(4, 5));

        ed.push_back(std::pair<int,int>(1, 9));
        ed.push_back(std::pair<int,int>(2, 6));
        ed.push_back(std::pair<int,int>(3, 7));
        ed.push_back(std::pair<int,int>(4, 10));
        ed.push_back(std::pair<int,int>(6, 7));
        ed.push_back(std::pair<int,int>(9, 10));

        ed.push_back(std::pair<int,int>(0, 11));
        ed.push_back(std::pair<int,int>(5, 13));
        ed.push_back(std::pair<int,int>(11, 12));
        ed.push_back(std::pair<int,int>(12, 13));
        ed.push_back(std::pair<int,int>(11, 19));
        ed.push_back(std::pair<int,int>(13, 24));

        ed.push_back(std::pair<int,int>(14, 20));
        ed.push_back(std::pair<int,int>(14, 15));
        ed.push_back(std::pair<int,int>(15, 23));
        ed.push_back(std::pair<int,int>(17, 21));
        ed.push_back(std::pair<int,int>(17, 18));
        ed.push_back(std::pair<int,int>(18, 22));

        ed.push_back(std::pair<int,int>(19, 20));
        ed.push_back(std::pair<int,int>(20, 21));
        ed.push_back(std::pair<int,int>(21, 22));
        ed.push_back(std::pair<int,int>(22, 23));
        ed.push_back(std::pair<int,int>(23, 24));

    }


}








