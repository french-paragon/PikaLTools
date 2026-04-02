#include <QTest>

#include "processing/relativeoffsetsestimator.h"

#include <random>

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

std::vector<int> generateShuffledIdxs(int nPoints, int shuffleRadius, int seed = 42) {
    std::vector<int> ret(nPoints);

    std::default_random_engine re;
    re.seed(seed);

    for (int i = 0; i < nPoints; i++) {
        ret[i] = i;
    }

    std::uniform_int_distribution advanceDist(1,shuffleRadius);

    int prev = 0;
    int next = std::min(nPoints-1, prev + advanceDist(re));

    while (prev < nPoints-1) {
        std::shuffle(ret.data()+prev, ret.data()+next, re); //points cannot be shuffled more than searchRadius position away from their original position
        prev = next+1;
        next = std::min(nPoints-1, prev + advanceDist(re));
    }

    #ifndef NDEBUG
    for (int i = 0; i < nPoints; i++) {
        assert(std::abs(ret[i] - i) <= shuffleRadius);
    }
    #endif

    return ret;
}

class TestPushBroomRelativeOffsetsEstimators : public QObject {

    Q_OBJECT
private Q_SLOTS:

    void initTestCase();

    void testBayesianLineEstimator();
    void testGlobalEstimatorHorizontalBehavior();
    void testVerticalReordererBehavior();
    void testComputeVerticallyReorderedImage();
    void testTSPGreedyHeuristic();
    void testGlobalEstimatorVerticalBehavior();
    void testHorizontallyRectifiedVerticallyReorderedImage();
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

    auto accumulated = PikaLTools::PushBroomRelativeOffsets::computeAccumulatedFromRelativeShifts(rectificationShifted.value());

    qInfo() << "Convergence shifted: " << rectificationShifted.convergenceStr().c_str();
    qInfo() << "Shifts dx data shifted: " << infos.join(", ");

    infos.clear();
    infos.reserve(accumulated.accumulatedShifts.size());

    for (auto const& shift_x : accumulated.accumulatedShifts) {
        infos << QString("[shift x: %1]").arg(shift_x);
    }
    qInfo() << "Shifts x data shifted: " << infos.join(", ");

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

    QStringList infos;
    infos.reserve(rectificationDataPriorsOnly.value().size());

    for (auto const& shiftInfos : rectificationDataPriorsOnly.value()) {
        infos << QString("[dx: %1, y: %2]").arg(shiftInfos.dx).arg(shiftInfos.y);
    }

    qInfo() << "Convergence priors only: " << rectificationDataPriorsOnly.convergenceStr().c_str();
    qInfo() << "Shifts data priors only: " << infos.join(", ");

    for (int i = 0; i < rectificationDataPriorsOnly.value().size(); i++) {
        auto const& shiftInfos = rectificationDataPriorsOnly.value()[i];
        QVERIFY(std::abs(shiftInfos.dx) < 1e-3);
        QVERIFY(std::abs(shiftInfos.y - i) < 1e-3);
    }

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

void TestPushBroomRelativeOffsetsEstimators::testVerticalReordererBehavior() {


    constexpr int nLines = 7, nSamples = 1, nChannels = 1;
    constexpr std::array<int,nLines> disorderedIndicesR1{0,2,1,3,5,4,6};
    constexpr std::array<int,nLines> disorderedIndicesR2{0,3,2,1,6,4,5};
    constexpr std::array<int,nLines> disorderedIndicesR3{1,0,2,6,5,4,3};

    constexpr std::array<int,nLines> disorderedInvIndicesR1{0,2,1,3,5,4,6};
    constexpr std::array<int,nLines> disorderedInvIndicesR2{0,3,2,1,5,6,4};
    constexpr std::array<int,nLines> disorderedInvIndicesR3{1,0,2,6,5,4,3};

    Multidim::Array<float,3> frameUnshifted(nLines, nSamples, nChannels);

    for (int i = 0; i < nLines; i++) {
        float factor = float(i) / float(nLines-1);
        frameUnshifted.atUnchecked(i,0,0) = factor;
    }

    Multidim::Array<float,3> frame(nLines, nSamples, nChannels);

    //Maximal radius = 1
    for (int i = 0; i < nLines; i++) {
        frame.atUnchecked(i,0,0) = frameUnshifted.atUnchecked(disorderedIndicesR1[i],0,0);
    }

    std::vector<float> horizontalShifts(nLines);
    std::fill(horizontalShifts.begin(), horizontalShifts.end(), 0);
    constexpr int hWindow = 1;
    int searchRadius = 1;

    std::vector<int> orderEstimated = PikaLTools::PushBroomRelativeOffsets::estimatePushBroomVerticalReorderBayesian(frame,
                                                                                                                     horizontalShifts,
                                                                                                                     hWindow,
                                                                                                                     searchRadius,
                                                                                                                     linesDim,
                                                                                                                     samplesDim,
                                                                                                                     channelsDim);

    QStringList orderStr;
    for (int idx : orderEstimated) {
        orderStr << QString("%1").arg(idx);
    }
    QString orderEstimatedR1Str = "orderEstimated R1: [" + orderStr.join(", ") + "]";

    qInfo() << orderEstimatedR1Str;

    QCOMPARE(orderEstimated.size(), nLines);

    for (int i = 0; i < nLines; i++) {
        QCOMPARE(orderEstimated[i], disorderedInvIndicesR1[i]);
    }

    //Maximal radius = 2
    for (int i = 0; i < nLines; i++) {
        frame.atUnchecked(i,0,0) = frameUnshifted.atUnchecked(disorderedIndicesR2[i],0,0);
    }
    searchRadius = 2;

    orderEstimated = PikaLTools::PushBroomRelativeOffsets::estimatePushBroomVerticalReorderBayesian(frame,
                                                                                                    horizontalShifts,
                                                                                                    hWindow,
                                                                                                    searchRadius,
                                                                                                    linesDim,
                                                                                                    samplesDim,
                                                                                                    channelsDim);

    orderStr.clear();
    for (int idx : orderEstimated) {
        orderStr << QString("%1").arg(idx);
    }
    QString orderEstimatedR2Str = "orderEstimated R2: [" + orderStr.join(", ") + "]";

    qInfo() << orderEstimatedR2Str;

    QCOMPARE(orderEstimated.size(), nLines);

    for (int i = 0; i < nLines; i++) {
        QCOMPARE(orderEstimated[i], disorderedInvIndicesR2[i]);
    }

    //Maximal radius = 3
    for (int i = 0; i < nLines; i++) {
        frame.atUnchecked(i,0,0) = frameUnshifted.atUnchecked(disorderedIndicesR3[i],0,0);
    }
    searchRadius = 3;

    orderEstimated = PikaLTools::PushBroomRelativeOffsets::estimatePushBroomVerticalReorderBayesian(frame,
                                                                                                    horizontalShifts,
                                                                                                    hWindow,
                                                                                                    searchRadius,
                                                                                                    linesDim,
                                                                                                    samplesDim,
                                                                                                    channelsDim);

    orderStr.clear();
    for (int idx : orderEstimated) {
        orderStr << QString("%1").arg(idx);
    }
    QString orderEstimatedR3Str = "orderEstimated R3: [" + orderStr.join(", ") + "]";

    qInfo() << orderEstimatedR3Str;

    QCOMPARE(orderEstimated.size(), nLines);

    for (int i = 0; i < nLines; i++) {
        QCOMPARE(orderEstimated[i], disorderedInvIndicesR3[i]);
    }

}
void TestPushBroomRelativeOffsetsEstimators::testComputeVerticallyReorderedImage() {

    constexpr int nPoints = 7, nSamples = 1, nChannels = 1;
    constexpr int searchRadius = nPoints;

    Multidim::Array<int,3> frameUnshifted(nPoints, nSamples, nChannels);
    Multidim::Array<int,3> frameShifted(nPoints, nSamples, nChannels);

    std::vector<int> shuffleIdxs = generateShuffledIdxs(nPoints, searchRadius);

    for (int i = 0; i < nPoints; i++) {
        frameUnshifted.atUnchecked(i,0,0) = i;
        frameShifted.atUnchecked(i,0,0) = shuffleIdxs[i];
    }

    Multidim::Array<int,3> reordered =
        PikaLTools::PushBroomRelativeOffsets::computeVerticallyReorderedImage(frameShifted,
                                                                          shuffleIdxs,
                                                                          linesDim,
                                                                          samplesDim,
                                                                          channelsDim);

    for (int i = 0; i < nPoints; i++) {
        QCOMPARE(reordered.atUnchecked(i,0,0), i);
    }


}

void TestPushBroomRelativeOffsetsEstimators::testTSPGreedyHeuristic() {

    int nPoints = 100;

    int searchRadius = 25;

    std::vector<float> positions(nPoints);
    std::vector<int> shuffleIdxs = generateShuffledIdxs(nPoints, searchRadius);
    std::vector<float> shuffled_positions(nPoints);
    std::vector<int> shuffled_idxs(nPoints);

    for (int i = 0; i < nPoints; i++) {
        float pos = float(i)/4 + std::sin(4*M_PI*float(i)/nPoints);
        positions[i] = pos;
    }

    #ifndef NDEBUG
    for (int i = 1; i < nPoints; i++) {
        assert(positions[i] > positions[i-1]);
    }
    #endif

    for (int i = 0; i < nPoints; i++) {
        shuffled_positions[shuffleIdxs[i]] = positions[i];
        shuffled_idxs[shuffleIdxs[i]] = i;
    }

    #ifndef NDEBUG
    std::vector<int> unshuffled_idxs(nPoints);

    for (int i = 0; i < nPoints; i++) {
        unshuffled_idxs[i] = shuffled_idxs[shuffleIdxs[i]];
        assert(unshuffled_idxs[i] == i);
    }
    std::vector<int> sorting_idxs(nPoints);

    for (int i = 0; i < nPoints; i++) {
        sorting_idxs[i] = i;
    }

    std::sort(sorting_idxs.begin(), sorting_idxs.end(), [&shuffled_positions] (int i1, int i2) {
        return shuffled_positions[i1] < shuffled_positions[i2];
    });

    for (int i = 0; i < nPoints; i++) {
        assert(sorting_idxs[i] == shuffleIdxs[i]);
    }

    #endif

    int nComps = std::min(nPoints-1,2*searchRadius);

    Multidim::Array<float,2> costs(nPoints, nComps);

    for (int i = 0; i < nPoints; i++) {
        for (int c = 0; c < nComps; c++) {
            float dist = std::numeric_limits<float>::infinity();

            int j = i+c+1;

            if (j < nPoints) {
                dist = std::abs(shuffled_positions[i] - shuffled_positions[j]);
            }

            costs.atUnchecked(i,c) = dist;
        }
    }

    std::vector<int> orderEstimated = PikaLTools::PushBroomRelativeOffsets::radiusLimitedGreedyTsp(costs);

    if (orderEstimated.back() < orderEstimated.front()) {
        std::reverse(orderEstimated.begin(), orderEstimated.end());
    }

    QCOMPARE(orderEstimated.size(), nPoints);

    for (int i = 0; i < nPoints; i++) {
        QCOMPARE(orderEstimated[i], shuffleIdxs[i]);
    }

    Multidim::Array<float,3> frameShifted(nPoints, 1, 1);

    for (int i = 0; i < nPoints; i++) {
        frameShifted.atUnchecked(i,0,0) = shuffled_positions[i];
    }

    Multidim::Array<float,3> reordered =
        PikaLTools::PushBroomRelativeOffsets::computeVerticallyReorderedImage(frameShifted,
                                                                              orderEstimated,
                                                                              linesDim,
                                                                              samplesDim,
                                                                              channelsDim);

    for (int i = 0; i < nPoints; i++) {
        QCOMPARE(reordered.valueUnchecked(i,0,0), positions[i]);
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


void TestPushBroomRelativeOffsetsEstimators::testHorizontallyRectifiedVerticallyReorderedImage() {

    constexpr int nPoints = 7, nSamples = 1, nChannels = 1;
    constexpr int searchRadius = nPoints;

    Multidim::Array<int,3> frameUnshifted(nPoints, nSamples, nChannels);
    Multidim::Array<int,3> frameShifted(nPoints, nSamples, nChannels);

    std::vector<int> shuffleIdxs = generateShuffledIdxs(nPoints, searchRadius);

    for (int i = 0; i < nPoints; i++) {
        frameUnshifted.atUnchecked(i,0,0) = i;
        frameShifted.atUnchecked(i,0,0) = shuffleIdxs[i];
    }

    std::vector<float> dxs(nPoints);

    std::fill(dxs.begin(), dxs.end(), 0);

    Multidim::Array<int,3> reordered =
        PikaLTools::PushBroomRelativeOffsets::computeHorizontallyRectifiedVerticallyReorderedImage
        (
        frameShifted,
        dxs,
        shuffleIdxs,
        linesDim,
        samplesDim,
        channelsDim);

    for (int i = 0; i < nPoints; i++) {
        QCOMPARE(reordered.atUnchecked(i,0,0), i);
    }

}
QTEST_MAIN(TestPushBroomRelativeOffsetsEstimators)

#include "testPushBroomRelativeOffsetsEstimators.moc"
