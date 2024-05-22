#include "libs/io/read_envi_bil.h"
#include "libs/geo/coordinate_conversions.h"

#include <StereoVision/QImageDisplayWidget/imagewindow.h>
#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <QApplication>
#include <QTextStream>
#include <QVector>

#include <StereoVision/utils/types_manipulations.h>

#define QCUSTOMPLOT_USE_LIBRARY
#include "qcustomplot.h"

struct timedPositions {
    CartesianCoord<double> pos;
    double time;
};

QVector<timedPositions> getINSMotionValue(QVector<QString> const& files) {

    QVector<timedPositions> accumulated;

    double meanLat = 0;
    double meanLon = 0;

    for (QString const& file : files) {
        std::vector<EnviBilLcfLine> lines = read_envi_bil_lcf_data(file.toStdString());

        accumulated.reserve(accumulated.size() + lines.size());

        for (EnviBilLcfLine const& line : lines) {

            double lat= line.lat;
            double lon = line.lon;
            double alt = line.height;

            meanLat += lat;
            meanLon += lon;

            accumulated.push_back({convertLatLonToECEF<double>(lat, lon, alt), line.timeStamp});
        }
    }

    meanLat /= accumulated.size();
    meanLon /= accumulated.size();

    StereoVision::Geometry::AffineTransform<double> ecef2local = getLocalFrameAtPos<double>(meanLat, meanLon);

    for (timedPositions & pos : accumulated) {

        Eigen::Matrix<double,3,1> homogeneous;
        homogeneous[0] = pos.pos.x;
        homogeneous[1] = pos.pos.y;
        homogeneous[2] = pos.pos.z;

        Eigen::Matrix<double,3,1> local = ecef2local*homogeneous;
        pos.pos.x = local[0];
        pos.pos.y = local[1];
        pos.pos.z = local[2];
    }

    return accumulated;
}

int main(int argc, char** argv) {

    QTextStream out(stdout);

    QApplication app(argc, argv);

    QVector<QString> filenames(argc-1);

    for (int i = 1; i < argc; i++) {
        filenames[i-1] = QString(argv[i]);
    }

    QVector<timedPositions> ins_signal = getINSMotionValue(filenames);

    QVector<double> posx_values(ins_signal.size());
    QVector<double> posy_values(ins_signal.size());
    QVector<double> posz_values(ins_signal.size());
    QVector<double> timestamps(ins_signal.size());
    QVector<double> frameNumbers(ins_signal.size());

    for (int i = 0; i < ins_signal.size(); i++) {

        frameNumbers[i] = i;
        posx_values[i] = ins_signal[i].pos.x;
        posy_values[i] = ins_signal[i].pos.y;
        posz_values[i] = ins_signal[i].pos.z;
        timestamps[i] = ins_signal[i].time;
    }

    QCustomPlot tzPlot;
    tzPlot.resize(960, 600);

    QCPGraph* graph_pos = tzPlot.addGraph();

    graph_pos->setData(frameNumbers, posz_values);
    graph_pos->setPen(QColor(0,0,255));

    graph_pos->rescaleAxes();

    tzPlot.xAxis->setLabel("t");
    tzPlot.yAxis->setLabel("z");

    tzPlot.setWindowTitle("Z as a function of time");

    tzPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    tzPlot.replot();
    tzPlot.show();

    QCustomPlot tyPlot;
    tyPlot.resize(960, 600);

    graph_pos = tyPlot.addGraph();

    graph_pos->setData(frameNumbers, posy_values);
    graph_pos->setPen(QColor(0,255,0));

    graph_pos->rescaleAxes();

    tyPlot.xAxis->setLabel("t");
    tyPlot.yAxis->setLabel("y");

    tyPlot.setWindowTitle("Y as a function of time");

    tyPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    tyPlot.replot();
    tyPlot.show();

    QCustomPlot txPlot;
    txPlot.resize(960, 600);

    graph_pos = txPlot.addGraph();

    graph_pos->setData(frameNumbers, posx_values);
    graph_pos->setPen(QColor(255,0,0));

    graph_pos->rescaleAxes();

    txPlot.xAxis->setLabel("t");
    txPlot.yAxis->setLabel("x");

    txPlot.setWindowTitle("X as a function of time");

    txPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    txPlot.replot();
    txPlot.show();

    QCustomPlot xyPlot;
    xyPlot.resize(960, 600);

    graph_pos = xyPlot.addGraph();

    graph_pos->setData(posx_values, posy_values);
    graph_pos->setPen(QColor(255,255,0));

    graph_pos->rescaleAxes();

    xyPlot.xAxis->setLabel("x");
    xyPlot.yAxis->setLabel("y");

    xyPlot.setWindowTitle("XY plane trajectory");

    xyPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    xyPlot.replot();
    xyPlot.show();

    QCustomPlot xzPlot;
    xzPlot.resize(960, 600);

    graph_pos = xzPlot.addGraph();

    graph_pos->setData(posx_values, posz_values);
    graph_pos->setPen(QColor(255,0,255));

    graph_pos->rescaleAxes();

    xzPlot.xAxis->setLabel("x");
    xzPlot.yAxis->setLabel("z");

    xzPlot.setWindowTitle("XZ plane trajectory");

    xzPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    xzPlot.replot();
    xzPlot.show();

    QCustomPlot yzPlot;
    yzPlot.resize(960, 600);

    graph_pos = yzPlot.addGraph();

    graph_pos->setData(posy_values, posz_values);
    graph_pos->setPen(QColor(0,255,255));

    graph_pos->rescaleAxes();

    yzPlot.xAxis->setLabel("y");
    yzPlot.yAxis->setLabel("z");

    yzPlot.setWindowTitle("YZ plane trajectory");

    yzPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    yzPlot.replot();
    yzPlot.show();

    return app.exec();
}
