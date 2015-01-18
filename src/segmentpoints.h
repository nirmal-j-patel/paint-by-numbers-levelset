#ifndef __SEGMENTPOINTS_H__
#define __SEGMENTPOINTS_H__

#include <QPoint>
#include <QPair>
#include <QVector>
#include <cmath>

#include <stdexcept>

#include <itkImageRegionIteratorWithIndex.h>
#include <itkImageFileReader.h>


//typedef itk::Image<size_t, 2> LabelImageType;


struct Segment {
    unsigned int color;
    QVector<QPoint> points;
};

struct SegmentWithBucketIndex {
    QPoint p1_bucket, p2_bucket;
    QPoint p1, p2;
};

inline float distance(const QPoint &p1, const QPoint &p2) {
    return sqrt( pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) );
}

inline float slope(const QPoint &p1, const QPoint &p2) {
    return (p2.y() - p1.y()) / (p2.x() - p1.x());
}

inline bool isInNeighborhood(const QPoint &p1, const QPoint &p2, const unsigned int &radius=0) {
    return abs(p1.x() - p2.x()) <= radius & abs(p1.y() - p2.y()) <= radius;
}


QPair<QPoint, QPoint> findFarthestPoints(const QVector<QPoint> &points) {
    QPair<QPoint, QPoint> maxDistance;
    maxDistance.first = points[0];
    maxDistance.second = points[1];

    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < points.size(); j++) {
            if (i < j && distance(points[i], points[j]) > distance(maxDistance.first, maxDistance.second)
                    && distance(points[i], points[j]) < 50) {
                maxDistance.first = points[i];
                maxDistance.second = points[j];
            }
        }
    }

    return maxDistance;
}

//QVector<Segment> getSegments(LabelImageType *image) {
QVector<Segment> getSegments(itk::Image<size_t, 2>::Pointer image) {
    QVector <Segment> segments;

    //itk::ImageRegionConstIteratorWithIndex<LabelImageType> iIt(image, image->GetLargestPossibleRegion());
    itk::ImageRegionConstIteratorWithIndex<itk::Image<size_t, 2>> iIt(image, image->GetLargestPossibleRegion());

    iIt.GoToBegin();

    while(!iIt.IsAtEnd()) {
        auto bgcolor = iIt.Get();
        auto index = iIt.GetIndex();

        bool found = false;

        for (int i = 0; i < segments.size(); i++) {
            if (segments[i].color == bgcolor) {
                segments[i].points.append(QPoint(index[0], index[1]));

                found = true;
            }
        }

        if (!found) {
            Segment s;
            s.color = bgcolor;
            s.points.append(QPoint(index[0], index[1]));

            segments.append(s);
        }

        ++iIt;
    }

    return segments;

}

QVector<QPair<QPoint, QPoint>> getSegmentEndPoints(const QVector<Segment> &segments) {
    QVector<QPair<QPoint, QPoint>> finalSegments;

    for (int i = 0; i < segments.size(); i++) {
        if (segments[i].points.size() < 2 || segments[i].points.size() > 5000) {
            continue;
        }

        finalSegments.append(findFarthestPoints(segments[i].points));
    }

    return finalSegments;
}

QVector<QPair<QPoint, QPoint>> findConnections(QVector<QPair<QPoint, QPoint>> finalSegments, const size_t &bucketsize) {
    QVector<QPair<QPoint, QPoint>> connections;

    //const unsigned int bucketsize = 10; // 10x10 buckets
    QVector<SegmentWithBucketIndex> segment_with_bucket_index;

    for (int i = 0; i < finalSegments.size(); i++) {
        SegmentWithBucketIndex s;

        s.p1 = finalSegments[i].first;
        s.p2 = finalSegments[i].second;

        s.p1_bucket.setX(s.p1.x() / bucketsize);
        s.p1_bucket.setY(s.p1.y() / bucketsize);

        s.p2_bucket.setX(s.p2.x() / bucketsize);
        s.p2_bucket.setY(s.p2.y() / bucketsize);

        segment_with_bucket_index.append(s);
    }

    for (int i = 0; i < segment_with_bucket_index.size(); i++) {
        for (int j = 0; j < segment_with_bucket_index.size(); j++) {
            // if (i == j) {
            if (i <= j) {
                continue;
            } else {
                // variables starting with b indicate bucket, so b1_1 = bucket 1 point 1
                const QPoint b1_1 = segment_with_bucket_index[i].p1_bucket;
                const QPoint b1_2 = segment_with_bucket_index[i].p2_bucket;

                const QPoint b2_1 = segment_with_bucket_index[j].p1_bucket;
                const QPoint b2_2 = segment_with_bucket_index[j].p2_bucket;

                const QPoint p1_1 = segment_with_bucket_index[i].p1;
                const QPoint p1_2 = segment_with_bucket_index[i].p2;

                const QPoint p2_1 = segment_with_bucket_index[j].p1;
                const QPoint p2_2 = segment_with_bucket_index[j].p2;

                const int radius = 2;

                if (isInNeighborhood(b1_1, b2_1, radius)) {
                    connections << qMakePair(p1_1, p2_1);
                } else if (isInNeighborhood(b1_1, b2_2, radius)) {
                    connections << qMakePair(p1_1, p2_2);
                } else if (isInNeighborhood(b1_2, b2_1, radius)) {
                    connections << qMakePair(p1_2, p2_1);
                } else if (isInNeighborhood(b1_2, b2_2, radius)) {
                    connections << qMakePair(p1_2, p2_2);
                }
            }
        }
    }

    return connections;
}

QPoint findNext(const QPoint &current, const QPoint &prev, const QVector<QPair<QPoint, QPoint>> &segments, const QVector<QPair<QPoint, QPoint>> &connections) {
    static int i = 0;
    //qDebug() << "Found " << i++ << " points";

    for (int i = 0; i < segments.size(); i++) {
        const QPoint p1 = segments[i].first;
        const QPoint p2 = segments[i].second;
        if (current == p1 && prev != p2) {
            return p2;
        } else if (current == p2 && prev != p1) {
            return p1;
        } else {
            for (int j = 0; j < connections.size(); j++) {
                const QPoint c1 = connections[j].first;
                const QPoint c2 = connections[j].second;

                if (current == c1 && prev != c2) {
                    return c2;
                } else if (current == c2 && prev != c1) {
                    return c1;
                }
            }
        }
    }

    throw std::domain_error("Could not find next point");
}

QVector<QPoint> getContourPoints(const QPoint &begin, const QPoint &end, const QVector<QPair<QPoint, QPoint>> & finalSegments, const QVector<QPair<QPoint, QPoint>> &connections) {
    QVector<QPoint> points;

    // 506, 511 is a point on the border of a closed contour
    QPoint current = begin;
    QPoint prev = QPoint(-1, -1);
    QPoint tmp;


    points.clear();
    //points.push_back(begin);
    points.push_back(current);

    do {
        tmp = current;
        current = findNext(current, prev, finalSegments, connections);
        prev = tmp;

        points.push_back(current);
    } while (end != current);


    return points;
}

#endif
