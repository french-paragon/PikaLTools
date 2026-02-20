#include <QTest>

#include "processing/relativeoffsetsestimator.h"

constexpr int linesDim = 0, samplesDim = 1, channelsDim = 2;
constexpr int lines_test_frame_nLines = 7, lines_test_frame_nSamples = 5, lines_test_frame_nChannels = 3;

Multidim::Array<float,3> renderLinesTestFrame(std::array<int,lines_test_frame_nLines> shifts,
                                               int line1Id = 2, int line2Id = 4,
                                               std::array<float,lines_test_frame_nChannels> bgColor = {0,0,1},
                                               std::array<float,lines_test_frame_nChannels> line1Color = {1,0,0},
                                               std::array<float,lines_test_frame_nChannels> line2Color = {0,1,1}) {

    Multidim::Array<float,3> frame(lines_test_frame_nLines, lines_test_frame_nSamples, lines_test_frame_nChannels);

    for (int i = 0; i < lines_test_frame_nLines; i++) {
        int shift = shifts[i];

        for (int j = 0; j < lines_test_frame_nSamples; j++) {

            std::array<float,lines_test_frame_nChannels> color = bgColor;
            if (j + shift == line1Id) {
                color = line1Color;
            }
            if (j + shift == line2Id) {
                color = line2Color;
            }

            for (int c = 0; c < lines_test_frame_nChannels; c++) {
                frame.atUnchecked(i,j,c) = color[c];
            }
        }
    }

    return frame;

}

Multidim::Array<float,3> renderReorderedLinesTestFrame(std::array<int,lines_test_frame_nLines> horizontal_shifts,
                                                        std::array<int,lines_test_frame_nLines> vertical_idxs,
                                                        std::array<float,lines_test_frame_nChannels> bgColor = {0,0,1},
                                                        std::array<float,lines_test_frame_nChannels> startPointColor = {1,0,0},
                                                        std::array<float,lines_test_frame_nChannels> endPointColor = {0,1,1}) {

    auto spreadFunction = [] (int coord) {
        int range = lines_test_frame_nSamples/2;
        double x = coord - range;
        double s = range/2;
        return std::exp(-x*x/(s*s));
    };

    Multidim::Array<float,3> frame(lines_test_frame_nLines, lines_test_frame_nSamples, lines_test_frame_nChannels);

    for (int i = 0; i < lines_test_frame_nLines; i++) {
        int shift = horizontal_shifts[i];
        int idx = vertical_idxs[i];

        double lambda = 1 - double(i)/(lines_test_frame_nLines-1);

        for (int j = 0; j < lines_test_frame_nSamples; j++) {

            double spread = spreadFunction(j + shift);

            std::array<float,lines_test_frame_nChannels> color_s;
            std::array<float,lines_test_frame_nChannels> color_e;

            for (int c = 0; c < lines_test_frame_nChannels; c++) {
                color_s[c] = spread*startPointColor[c] + (1-spread)*bgColor[c];
                color_e[c] = spread*endPointColor[c] + (1-spread)*bgColor[c];

                frame.atUnchecked(idx,j,c) = lambda*color_s[c] + (1-lambda)*color_e[c];
            }
        }
    }

    return frame;

}

class TestPushBroomRelativeOffsetsEstimators : public QObject {

    Q_OBJECT
private Q_SLOTS:

    void initTestCase();

    void testBayesianLineEstimator();
    void testGlobalEstimatorHorizontalBehavior();
    void testGlobalEstimatorVerticalBehavior();
};


void TestPushBroomRelativeOffsetsEstimators::initTestCase() {

}

void TestPushBroomRelativeOffsetsEstimators::testBayesianLineEstimator() {

    constexpr int nLines = lines_test_frame_nLines, nSamples = lines_test_frame_nSamples, nChannels = lines_test_frame_nChannels;
    constexpr int line1Id = 2, line2Id = 4;
    constexpr std::array<int,nLines> shifts{0,-1,-1,0,1,1,0};
    constexpr std::array<int,nLines> shiftsNoShift{0,0,0,0,0,0,0};
    constexpr std::array<float,nChannels> bgColor{0,0,1};
    constexpr std::array<float,nChannels> line1Color{1,0,0};
    constexpr std::array<float,nChannels> line2Color{0,1,1};

    Multidim::Array<float,3> frameUnshifted = renderLinesTestFrame(shiftsNoShift, line1Id, line2Id, bgColor, line1Color, line2Color);
    Multidim::Array<float,3> frame = renderLinesTestFrame(shifts, line1Id, line2Id, bgColor, line1Color, line2Color);

    constexpr bool verbose = false;

    constexpr int hwindow = nSamples;
    constexpr double dx_pos_lambda = 0.01;

    constexpr int nIterations = 1000;
    constexpr double residual_treshold = 1e-5;

    auto rectificationNoShifts =
        PikaLTools::PushBroomRelativeOffsets::estimatePushBroomHorizontalShiftBayesian<verbose>
        (
            frameUnshifted,
            hwindow,
            dx_pos_lambda,
            nIterations,
            residual_treshold,
            linesDim,
            samplesDim,
            channelsDim);

    QVERIFY(rectificationNoShifts.convergence() != StereoVision::ConvergenceType::Failed);
    QCOMPARE(rectificationNoShifts.value().size(), shiftsNoShift.size()-1);

    QStringList infos;
    infos.reserve(rectificationNoShifts.value().size());

    for (auto const& dx : rectificationNoShifts.value()) {
        infos << QString("[dx: %1]").arg(dx);
    }

    qInfo() << "Convergence unshifted: " << rectificationNoShifts.convergenceStr().c_str();
    qInfo() << "Shifts data unshifted: " << infos.join(", ");

    for (int i = 0; i < rectificationNoShifts.value().size(); i++) {
        auto const& dx = rectificationNoShifts.value()[i];
        QVERIFY(std::abs(dx) < 1e-5);
    }

    auto rectificationShifted =
        PikaLTools::PushBroomRelativeOffsets::estimatePushBroomHorizontalShiftBayesian<verbose>
        (
            frame,
            hwindow,
            dx_pos_lambda,
            nIterations,
            residual_treshold,
            linesDim,
            samplesDim,
            channelsDim);

    QVERIFY(rectificationShifted.convergence() != StereoVision::ConvergenceType::Failed);
    QCOMPARE(rectificationShifted.value().size(), shifts.size()-1);

    infos.clear();
    infos.reserve(rectificationShifted.value().size());

    for (auto const& dx : rectificationShifted.value()) {
        infos << QString("[dx: %1]").arg(dx);
    }

    qInfo() << "Convergence shifted: " << rectificationShifted.convergenceStr().c_str();
    qInfo() << "Shifts data shifted: " << infos.join(", ");

    for (int i = 0; i < rectificationShifted.value().size(); i++) {
        auto const& dx = rectificationShifted.value()[i];
        QVERIFY(std::abs(dx - (shifts[i] - shifts[i+1])) < 1e-1);
    }

}

void TestPushBroomRelativeOffsetsEstimators::testGlobalEstimatorHorizontalBehavior() {

    constexpr int nLines = lines_test_frame_nLines, nSamples = lines_test_frame_nSamples, nChannels = lines_test_frame_nChannels;
    constexpr int line1Id = 2, line2Id = 4;
    constexpr std::array<int,nLines> shifts{0,-1,-1,0,1,1,0};
    constexpr std::array<int,nLines> shiftsNoShift{0,0,0,0,0,0,0};
    constexpr std::array<float,nChannels> bgColor{0,0,1};
    constexpr std::array<float,nChannels> line1Color{1,0,0};
    constexpr std::array<float,nChannels> line2Color{0,1,1};

    Multidim::Array<float,3> frameUnshifted = renderLinesTestFrame(shiftsNoShift, line1Id, line2Id, bgColor, line1Color, line2Color);
    Multidim::Array<float,3> frame = renderLinesTestFrame(shifts, line1Id, line2Id, bgColor, line1Color, line2Color);

    constexpr int hwindow = nSamples;
    constexpr int vradius = 1;
    constexpr double x_pos_lambda = 0.1;
    constexpr double y_pos_lambda = 1000000;
    constexpr double dx_pos_lambda = 0.01;
    constexpr double dy_pos_lambda = 0.1;
    constexpr double I_lambda_priors_only = 0;
    constexpr double I_lambda = 1;

    constexpr int nIterations = 1000;
    constexpr double residual_treshold = 1e-5;
    constexpr double damping_factor = 0.4;

    constexpr bool verbose = false;

    auto rectificationDataPriorsOnly =
        PikaLTools::PushBroomRelativeOffsets::estimateGlobalPushBroomPreRectification<verbose>
        (
            frame,
            hwindow,
            vradius,
            x_pos_lambda,
            y_pos_lambda,
            dx_pos_lambda,
            dy_pos_lambda,
            I_lambda_priors_only,
            nIterations,
            residual_treshold,
            damping_factor,
            linesDim,
            samplesDim,
            channelsDim);

    QVERIFY(rectificationDataPriorsOnly.convergence() != StereoVision::ConvergenceType::Failed);

    for (int i = 0; i < rectificationDataPriorsOnly.value().size(); i++) {
        auto const& shiftInfos = rectificationDataPriorsOnly.value()[i];
        QVERIFY(std::abs(shiftInfos.dx) < 1e-5);
        QVERIFY(std::abs(shiftInfos.y - i) < 1e-5);
    }

    QStringList infos;
    infos.reserve(rectificationDataPriorsOnly.value().size());

    for (auto const& shiftInfos : rectificationDataPriorsOnly.value()) {
        infos << QString("[dx: %1, y: %2]").arg(shiftInfos.dx).arg(shiftInfos.y);
    }

    qInfo() << "Convergence priors only: " << rectificationDataPriorsOnly.convergenceStr().c_str();
    qInfo() << "Shifts data priors only: " << infos.join(", ");

    auto rectificationDataUnshifted =
        PikaLTools::PushBroomRelativeOffsets::estimateGlobalPushBroomPreRectification<verbose>
        (
            frameUnshifted,
            hwindow,
            vradius,
            x_pos_lambda,
            y_pos_lambda,
            dx_pos_lambda,
            dy_pos_lambda,
            I_lambda_priors_only,
            nIterations,
            residual_treshold,
            damping_factor,
            linesDim,
            samplesDim,
            channelsDim);

    QVERIFY(rectificationDataUnshifted.convergence() != StereoVision::ConvergenceType::Failed);

    for (int i = 0; i < rectificationDataUnshifted.value().size(); i++) {
        auto const& shiftInfos = rectificationDataUnshifted.value()[i];
        QVERIFY(std::abs(shiftInfos.dx) < 1e-2);
        QVERIFY(std::abs(shiftInfos.y - i) < 1e-2);
    }

    infos.clear();
    infos.reserve(rectificationDataUnshifted.value().size());

    for (auto const& shiftInfos : rectificationDataUnshifted.value()) {
        infos << QString("[dx: %1, y: %2]").arg(shiftInfos.dx).arg(shiftInfos.y);
    }

    qInfo() << "Convergence unshifted: " << rectificationDataUnshifted.convergenceStr().c_str();
    qInfo() << "Shifts data unshifted: " << infos.join(", ");

    auto rectificationData =
        PikaLTools::PushBroomRelativeOffsets::estimateGlobalPushBroomPreRectification<verbose>
        (
        frame,
        hwindow,
        vradius,
        x_pos_lambda,
        y_pos_lambda,
        dx_pos_lambda,
        dy_pos_lambda,
        I_lambda,
        nIterations,
        residual_treshold,
        damping_factor,
        linesDim,
        samplesDim,
        channelsDim);

    QVERIFY(rectificationData.convergence() != StereoVision::ConvergenceType::Failed);

    infos.clear();
    infos.reserve(rectificationData.value().size());

    for (auto const& shiftInfos : rectificationData.value()) {
        infos << QString("[dx: %1, y: %2]").arg(shiftInfos.dx).arg(shiftInfos.y);
    }

    qInfo() << "Convergences shifted: " << rectificationData.convergenceStr().c_str();
    qInfo() << "Shifts data shifted: " << infos.join(", ");

    for (int i = 0; i < rectificationData.value().size(); i++) {
        auto const& shiftInfos = rectificationData.value()[i];
        QVERIFY(std::abs(shiftInfos.dx + shifts[i]) < 1e-1);
        QVERIFY(std::abs(shiftInfos.y - i) < 1e-1);
    }
}

void TestPushBroomRelativeOffsetsEstimators::testGlobalEstimatorVerticalBehavior() {

    constexpr int nLines = lines_test_frame_nLines, nSamples = lines_test_frame_nSamples, nChannels = lines_test_frame_nChannels;
    constexpr std::array<int,nLines> shifts{0,-1,-1,0,1,1,0};
    constexpr std::array<int,nLines> shiftsNoShift{0,0,0,0,0,0,0};
    constexpr std::array<int,nLines> disorderedIndices{0,2,1,3,5,4,6};
    constexpr std::array<int,nLines> orderedIndices{0,1,2,3,4,5,6};
    constexpr std::array<float,nChannels> bgColor{0,0,1};
    constexpr std::array<float,nChannels> startColor{1,0,0};
    constexpr std::array<float,nChannels> endColor{0,1,1};

    Multidim::Array<float,3> frameUnshifted = renderReorderedLinesTestFrame(shiftsNoShift, orderedIndices, bgColor, startColor, endColor);
    Multidim::Array<float,3> frame = renderReorderedLinesTestFrame(shiftsNoShift, disorderedIndices, bgColor, startColor, endColor);

    constexpr int hwindow = nSamples;
    constexpr int vradius = 1;
    constexpr double x_pos_lambda = 0.1;
    constexpr double y_pos_lambda = 0.001;
    constexpr double dx_pos_lambda = 0.01;
    constexpr double dy_pos_lambda = 0.001;
    constexpr double I_lambda = 1;

    constexpr int nIterations = 1000;
    constexpr double residual_treshold = 1e-5;
    constexpr double damping_factor = 0.4;

    constexpr bool verbose = false;

    auto rectificationDataUnshifted =
        PikaLTools::PushBroomRelativeOffsets::estimateGlobalPushBroomPreRectification<verbose>
        (
            frameUnshifted,
            hwindow,
            vradius,
            x_pos_lambda,
            y_pos_lambda,
            dx_pos_lambda,
            dy_pos_lambda,
            I_lambda,
            nIterations,
            residual_treshold,
            damping_factor,
            linesDim,
            samplesDim,
            channelsDim);

    QVERIFY(rectificationDataUnshifted.convergence() != StereoVision::ConvergenceType::Failed);
    QCOMPARE(rectificationDataUnshifted.value().size(), nLines);

    QStringList infos;
    infos.reserve(rectificationDataUnshifted.value().size());

    for (auto const& shiftInfos : rectificationDataUnshifted.value()) {
        infos << QString("[dx: %1, y: %2]").arg(shiftInfos.dx).arg(shiftInfos.y);
    }

    qInfo() << "Convergence unshifted: " << rectificationDataUnshifted.convergenceStr().c_str();
    qInfo() << "Shifts data unshifted: " << infos.join(", ");

    std::array<int,nLines> order = orderedIndices;
    std::sort(order.begin(), order.end(), [&rectificationDataUnshifted] (int i1, int i2) {
        return rectificationDataUnshifted.value()[i1].y < rectificationDataUnshifted.value()[i2].y;
    });

    for (int i = 0; i < rectificationDataUnshifted.value().size(); i++) {
        QCOMPARE(order[i],orderedIndices[i]);
    }

    auto rectificationDataShifted =
        PikaLTools::PushBroomRelativeOffsets::estimateGlobalPushBroomPreRectification<verbose>
        (
            frame,
            hwindow,
            vradius,
            x_pos_lambda,
            y_pos_lambda,
            dx_pos_lambda,
            dy_pos_lambda,
            I_lambda,
            nIterations,
            residual_treshold,
            damping_factor,
            linesDim,
            samplesDim,
            channelsDim);

    QVERIFY(rectificationDataShifted.convergence() != StereoVision::ConvergenceType::Failed);
    QCOMPARE(rectificationDataShifted.value().size(), nLines);

    infos.clear();
    infos.reserve(rectificationDataShifted.value().size());

    for (auto const& shiftInfos : rectificationDataShifted.value()) {
        infos << QString("[dx: %1, y: %2]").arg(shiftInfos.dx).arg(shiftInfos.y);
    }

    qInfo() << "Convergence shifted: " << rectificationDataShifted.convergenceStr().c_str();
    qInfo() << "Shifts data shifted: " << infos.join(", ");

    order = orderedIndices;
    std::sort(order.begin(), order.end(), [&rectificationDataShifted] (int i1, int i2) {
        return rectificationDataShifted.value()[i1].y < rectificationDataShifted.value()[i2].y;
    });

    for (int i = 0; i < rectificationDataShifted.value().size(); i++) {
        QCOMPARE(order[i],disorderedIndices[i]);
    }
}


QTEST_MAIN(TestPushBroomRelativeOffsetsEstimators)

#include "testPushBroomRelativeOffsetsEstimators.moc"
