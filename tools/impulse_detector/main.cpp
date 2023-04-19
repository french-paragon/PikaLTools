#include "libs/io/read_envi_bil.h"

#include <qImageDisplayWidget/imagewindow.h>
#include <qImageDisplayWidget/imageadapter.h>

#include <QApplication>
#include <QTextStream>
#include <QVector>

#include <LibStevi/utils/types_manipulations.h>

#define QCUSTOMPLOT_USE_LIBRARY
#include "qcustomplot.h"

struct timedData {
    double value;
    double time;
};

QVector<timedData> getMeanImageValue(QVector<QString> const& files) {

    QTextStream out(stdout);

    constexpr int linesAxis = 0;
    constexpr int samplesAxis = 1;
    constexpr int bandsAxis = 2;

    QVector<timedData> ret;

    std::array<int, 3> shape;
    Multidim::Array<float, 3> current_img;
    Multidim::Array<float, 3> previous_img;

    int sc = 0;

    for (QString const& file : files) {
        out << "Treating file: " << file << Qt::endl;

        current_img = read_envi_bil_to_float(file.toStdString());
        std::vector<double> times = read_envi_bil_times(file.toStdString());

        shape = current_img.shape();

        if (current_img.empty()) {
            QVector<timedData>();
        }

        if (times.size() != shape[linesAxis]) {
            QVector<timedData>();
        }

        if (previous_img.empty()) {

            std::array<int, 3> pShape = shape;
            pShape[linesAxis] = 1;
            previous_img = Multidim::Array<float, 3>(pShape);

            for (int i = 0; i < shape[samplesAxis]; i++) {
                for (int j = 0; j < shape[bandsAxis]; j++) {
                    std::array<int, 3> idx;
                    idx[linesAxis] = 0;
                    idx[samplesAxis] = i;
                    idx[bandsAxis] = j;

                    previous_img.atUnchecked(idx) = current_img.atUnchecked(idx);
                }
            }
        }

        for (int l = 0; l < shape[linesAxis]; l++, sc++) {

            double abs_diff = 0;

            for (int i = 0; i < shape[samplesAxis]; i++) {
                for (int j = 0; j < shape[bandsAxis]; j++) {
                    std::array<int, 3> idx;
                    idx[linesAxis] = l;
                    idx[samplesAxis] = i;
                    idx[bandsAxis] = j;

                    std::array<int, 3> pidx = idx;
                    pidx[linesAxis] = (l > 0) ? l-1 : 0;

                    if (l > 0) {
                        abs_diff += std::abs(current_img.atUnchecked(pidx) - current_img.atUnchecked(idx));
                    } else {
                        abs_diff += std::abs(previous_img.atUnchecked(pidx) - current_img.atUnchecked(idx));
                    }
                }
            }

            abs_diff /= shape[samplesAxis]*shape[bandsAxis];

            ret.push_back({abs_diff, times[l]});
        }

        for (int i = 0; i < shape[samplesAxis]; i++) {
            for (int j = 0; j < shape[bandsAxis]; j++) {
                std::array<int, 3> idx;
                idx[linesAxis] = 0;
                idx[samplesAxis] = i;
                idx[bandsAxis] = j;

                std::array<int, 3> pidx = idx;
                pidx[linesAxis] = shape[linesAxis]-1;

                previous_img.atUnchecked(idx) = current_img.atUnchecked(pidx);
            }
        }
    }

    return ret;
}

struct motionAmplitudeTimedData {
    double angularValue;
    double motionValue;
    double time;
};

QVector<motionAmplitudeTimedData> getINSMotionValue(QVector<QString> const& files) {

    constexpr double approx_earth_radius =  6.3781e6;

    QVector<motionAmplitudeTimedData> accumulated;

    bool previousLineProcessed = false;
    EnviBilLcfLine previousLine;

    for (QString const& file : files) {
        std::vector<EnviBilLcfLine> lines = read_envi_bil_lcf_data(file.toStdString());

        accumulated.reserve(accumulated.size() + lines.size());

        for (EnviBilLcfLine const& line : lines) {

            if (!previousLineProcessed) {
                previousLine = line;
                previousLineProcessed = true;
                continue;
            }

            auto angleDiff = [] (double angle1, double angle2) -> double {

                double a1 = angle1;
                if (a1 < 0) {
                    a1 += 2*M_PI;
                }

                double a2 = angle2;
                if (a2 < 0) {
                    a2 += 2*M_PI;
                }

                return a1-a2;
            };

            double roll_delta = angleDiff(line.roll, previousLine.roll);
            double pitch_delta = angleDiff(line.pitch, previousLine.pitch);
            double yaw_delta = angleDiff(line.yaw, previousLine.yaw);

            double delta_angle = std::sqrt(
                    roll_delta*roll_delta +
                    pitch_delta*pitch_delta +
                    yaw_delta*yaw_delta);

            double lat_delta = (line.lat - previousLine.lat)*approx_earth_radius/180.*M_PI;
            double lon_delta = (line.lon - previousLine.lon)/cos(line.lat/180*M_PI)*approx_earth_radius/180.*M_PI;
            double height_delta = line.height - previousLine.height;

            double delta_dist = std::sqrt(
                        lat_delta*lat_delta +
                        lon_delta*lon_delta +
                        height_delta*height_delta);

            previousLine = line;

            accumulated.push_back({delta_angle, delta_dist, line.timeStamp});
        }
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

    QVector<timedData> signal = getMeanImageValue(filenames);

    QVector<double> values(signal.size()-1);
    QVector<double> times(signal.size()-1);

    QVector<double> dvalues(signal.size()-2);
    QVector<double> dtimes(signal.size()-2);

    double minTime = std::numeric_limits<double>::infinity();
    double minValue = std::numeric_limits<double>::infinity();
    double minDValue = std::numeric_limits<double>::infinity();

    double maxTime = -std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    double maxDValue = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < signal.size()-1; i++) {
        timedData const& td = signal[i+1];

        values[i] = td.value;
        times[i] = i+1; //td.time;

        if (times[i] < minTime) {
            minTime = times[i];
        }

        if (times[i] > maxTime) {
            maxTime = times[i];
        }

        if (td.value < minValue) {
            minValue = td.value;
        }

        if (td.value > maxValue) {
            maxValue = td.value;
        }

        if (i > 0) {
            dvalues[i-1] = values[i] - values[i-1];
            dtimes[i-1] = times[i];

            if (dvalues[i-1] < minDValue) {
                minDValue = dvalues[i-1];
            }

            if (dvalues[i-1] > maxDValue) {
                maxDValue = dvalues[i-1];
            }
        }
    }

    out << "Collected " << values.size() << " samples" << Qt::endl;

    QCustomPlot basePlot;
    basePlot.resize(960, 600);

    QCPGraph* graph = basePlot.addGraph();
    graph->setData(times, values);
    graph->setPen(QPen(Qt::blue));

    // give the axes some labels:
    basePlot.xAxis->setLabel("time");
    basePlot.yAxis->setLabel("impulse");

    basePlot.xAxis->setRange(minTime, maxTime);
    basePlot.yAxis->setRange(minValue, maxValue);

    basePlot.setWindowTitle("Absolute image changes signal");

    basePlot.setInteraction(QCP::iRangeZoom);
    basePlot.setInteraction(QCP::iRangeDrag);

    basePlot.replot();
    basePlot.show();

    QCustomPlot diffPlot;
    diffPlot.resize(960, 600);

    graph = diffPlot.addGraph();
    graph->setData(dtimes, dvalues);
    graph->setPen(QPen(Qt::red));

    diffPlot.xAxis->setLabel("time");
    diffPlot.yAxis->setLabel("impulse delta");

    diffPlot.xAxis->setRange(minTime, maxTime);
    diffPlot.yAxis->setRange(minDValue, maxDValue);

    diffPlot.setWindowTitle("Changes in absolute image changes signal");

    diffPlot.setInteraction(QCP::iRangeZoom);
    diffPlot.setInteraction(QCP::iRangeDrag);

    diffPlot.replot();
    diffPlot.show();

    QVector<motionAmplitudeTimedData> ins_signal = getINSMotionValue(filenames);

    QVector<double> pos_values(ins_signal.size());
    QVector<double> angle_values(ins_signal.size());
    QVector<double> timestamps(ins_signal.size());
    QVector<double> frameNumbers(ins_signal.size());

    for (int i = 0; i < ins_signal.size(); i++) {

        frameNumbers[i] = i;
        pos_values[i] = ins_signal[i].motionValue;
        angle_values[i] = ins_signal[i].angularValue;
        timestamps[i] = ins_signal[i].time;
    }

    QCustomPlot insPlot;
    insPlot.resize(960, 600);

    QCPGraph* graph_dist = insPlot.addGraph();
    QCPGraph* graph_angle = insPlot.addGraph();

    graph_dist->setData(frameNumbers, pos_values);
    graph_dist->setPen(QColor(0,120,60));
    graph_dist->setName("Position data");

    graph_angle->setData(frameNumbers, angle_values);
    graph_angle->setPen(QColor(50,200,100));
    graph_angle->setName("Angular data");

    QObject::connect(diffPlot.xAxis, static_cast<void (QCPAxis::*)(const QCPRange &)>(&QCPAxis::rangeChanged),
                     diffPlot.xAxis2, static_cast<void (QCPAxis::*)(const QCPRange &)>(&QCPAxis::setRange));
    QObject::connect(diffPlot.yAxis, static_cast<void (QCPAxis::*)(const QCPRange &)>(&QCPAxis::rangeChanged),
                     diffPlot.yAxis2, static_cast<void (QCPAxis::*)(const QCPRange &)>(&QCPAxis::setRange));

    graph_dist->rescaleAxes();
    graph_angle->rescaleAxes(true);

    insPlot.xAxis->setLabel("time");
    insPlot.yAxis->setLabel("signal");

    insPlot.setWindowTitle("Change in INS/IMU signals");

    insPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    insPlot.legend->setVisible(true);

    insPlot.replot();
    insPlot.show();

    return app.exec();
}
