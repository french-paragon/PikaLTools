#include <QTest>

#include "processing/relativeoffsetsestimator.h"

class TestPushBroomRelativeOffsetsEstimators : public QObject {

    Q_OBJECT
private Q_SLOTS:

    void initTestCase();

    void testGlobalEstimator();
};


void TestPushBroomRelativeOffsetsEstimators::initTestCase() {

}

void TestPushBroomRelativeOffsetsEstimators::testGlobalEstimator() {

    constexpr int linesDim = 0, samplesDim = 1, channelsDim = 2;
    constexpr int nLines = 7, nSamples = 5, nChannels = 3;
    constexpr int line1Id = 2, line2Id = 4;
    constexpr std::array<int,nLines> shifts{0,-1,-1,0,1,1,0};
    //constexpr std::array<int,nLines> shifts{0,0,0,0,0,0,0};
    constexpr std::array<float,nChannels> bgColor{0,0,1};
    constexpr std::array<float,nChannels> line1Color{1,0,0};
    constexpr std::array<float,nChannels> line2Color{0,1,1};

    Multidim::Array<float,3> frameUnshifted(nLines, nSamples, nChannels);
    Multidim::Array<float,3> frame(nLines, nSamples, nChannels);

    for (int i = 0; i < nLines; i++) {
        int shift = shifts[i];

        for (int j = 0; j < nSamples; j++) {

            std::array<float,nChannels> color = bgColor;
            if (j + shift == line1Id) {
                color = line1Color;
            }
            if (j + shift == line2Id) {
                color = line2Color;
            }

            std::array<float,nChannels> colorUnshifted = bgColor;
            if (j == line1Id) {
                colorUnshifted = line1Color;
            }
            if (j == line2Id) {
                colorUnshifted = line2Color;
            }

            for (int c = 0; c < nChannels; c++) {
                frame.atUnchecked(i,j,c) = color[c];
                frameUnshifted.atUnchecked(i,j,c) = colorUnshifted[c];
            }
        }
    }

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
        QVERIFY(std::abs(shiftInfos.dx - shifts[i]) < 1e-1);
        QVERIFY(std::abs(shiftInfos.y - i) < 1e-1);
    }
}



QTEST_MAIN(TestPushBroomRelativeOffsetsEstimators)

#include "testPushBroomRelativeOffsetsEstimators.moc"
