#ifndef READ_TRAJECTORY_DATA_H
#define READ_TRAJECTORY_DATA_H

#include <vector>
#include <string>

struct RawTrajectoryLine {
    int nLine;
    double timeStamp;
    double roll;
    double pitch;
    double yaw;
    double lat;
    double lon;
    double height;
};

struct TrajectoryColumnsInfos {

    TrajectoryColumnsInfos() :
        timeCol(0),
        latCol(5),
        lonCol(4),
        heightCol(6),
        rollCol(1),
        pitchCol(2),
        yawCol(3)
    {

    }

    int timeCol;
    int latCol;
    int lonCol;
    int heightCol;
    int rollCol;
    int pitchCol;
    int yawCol;
};

std::vector<RawTrajectoryLine> read_trajectory_data(std::string const& filename,
                                                    TrajectoryColumnsInfos const& trajectoryDef = TrajectoryColumnsInfos(),
                                                    std::vector<std::string> const& separators = {" ", "\t"},
                                                    std::string const& commentSign = "#");

#endif // READ_TRAJECTORY_DATA_H
