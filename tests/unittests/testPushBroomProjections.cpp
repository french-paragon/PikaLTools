#include <QTest>

#include "processing/pushbroomprojections.h"

#include <LibStevi/geometry/rotations.h>

#include <set>

class TestPushBroomProjections : public QObject {

    Q_OBJECT
private Q_SLOTS:

    void initTestCase();

    void testTerrainPixelsSeenByScannerLine();
    void testTerrainPixelsNotSeenByScannerLine();
};

void TestPushBroomProjections::initTestCase() {

}

void TestPushBroomProjections::testTerrainPixelsSeenByScannerLine() {

    int camWidth = 100;
    float camFLen = 100;
    float camPP = (camWidth - 1.)/2;

    Multidim::Array<float, 2> terrain(camWidth, camWidth);

    for (int i = 0; i < camWidth; i++) {
        for (int j = 0; j < camWidth; j++) {
            terrain.atUnchecked(i,j) = 0; //basic init value of flat 0 terrain
        }
    }

    Eigen::Matrix3f cam2frame = Eigen::Matrix3f::Zero();

    cam2frame(0,0) = 1;
    cam2frame(1,1) = -1;
    cam2frame(2,2) = -1;

    Eigen::Vector3f rAxisFrame2Terrain(0,0,M_PI_2 + M_PI_4);
    Eigen::Matrix3f RMatFrame2Terrain = StereoVision::Geometry::rodriguezFormula(rAxisFrame2Terrain);

    Eigen::Vector3f framePos(camPP,camPP,std::sqrt(2)*camWidth);

    StereoVision::Geometry::AffineTransform<float> cam2terrain(RMatFrame2Terrain*cam2frame, framePos);

    float wrongMaxHeight = framePos.z()/10; //pretend the max height is above 0, so the raycasting can do a little bit of work!

    std::vector<std::array<int, 2>> pixelsList = PikaLTools::terrainPixelsSeenByScannerLine(terrain,
                                                                                            cam2terrain,
                                                                                            camFLen,
                                                                                            camPP,
                                                                                            camWidth,
                                                                                            wrongMaxHeight);

    std::vector<std::array<int, 2>> expectedList(camWidth);

    for (int i = 0; i < camWidth; i++) {
        expectedList[i] = {i, camWidth-i-1};
    }

    std::set<std::array<int, 2>> pixelsSet(pixelsList.begin(), pixelsList.end());

    QVERIFY(pixelsList.size() >= expectedList.size());

    for (int i = 0; i < camWidth; i++) {
        QVERIFY(pixelsSet.count(expectedList[i]) > 0);
    }

}

void TestPushBroomProjections::testTerrainPixelsNotSeenByScannerLine() {

    int camWidth = 100;
    float camFLen = 100;
    float camPP = (camWidth - 1.)/2;

    Multidim::Array<float, 2> terrain(camWidth, camWidth);

    for (int i = 0; i < camWidth; i++) {
        for (int j = 0; j < camWidth; j++) {
            terrain.atUnchecked(i,j) = 0; //basic init value of flat 0 terrain
        }
    }

    Eigen::Matrix3f cam2frame = Eigen::Matrix3f::Zero();

    cam2frame(0,0) = 1;
    cam2frame(1,1) = -1;
    cam2frame(2,2) = -1;

    Eigen::Vector3f rAxisFrame2Terrain(0,0,M_PI_2 + M_PI_4);
    Eigen::Matrix3f RMatFrame2Terrain = StereoVision::Geometry::rodriguezFormula(rAxisFrame2Terrain);

    Eigen::Vector3f framePos(camPP,camPP,-std::sqrt(2)*camWidth);

    StereoVision::Geometry::AffineTransform<float> cam2terrain(RMatFrame2Terrain*cam2frame, framePos);

    float wrongMaxHeight = framePos.z()/10; //pretend the max height is above 0, so the raycasting can do a little bit of work!

    std::vector<std::array<int, 2>> pixelsList = PikaLTools::terrainPixelsSeenByScannerLine(terrain,
                                                                                            cam2terrain,
                                                                                            camFLen,
                                                                                            camPP,
                                                                                            camWidth,
                                                                                            wrongMaxHeight);

    QCOMPARE(pixelsList.size(), 0);

}

QTEST_MAIN(TestPushBroomProjections)
#include "testPushBroomProjections.moc"
