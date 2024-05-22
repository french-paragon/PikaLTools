#include <QTest>

#include "geo/coordinate_conversions.h"

class TestGeoTools : public QObject {

    Q_OBJECT
private Q_SLOTS:

    void initTestCase();

    void testECEF2LocalFrame();
};

void TestGeoTools::initTestCase() {

}

void TestGeoTools::testECEF2LocalFrame() {

    constexpr int x_id = 0;
    constexpr int y_id = 1;
    constexpr int z_id = 2;

    float lat = 45;
    float lon = 10;
    float alt = 400;

    StereoVision::Geometry::AffineTransform<float> ecef2local1 = getLocalFrameAtPos(lat, lon, alt);
    StereoVision::Geometry::AffineTransform<float> local3ecef1 =
            StereoVision::Geometry::AffineTransform<float>(ecef2local1.R.transpose(), -ecef2local1.R.transpose()*ecef2local1.t);

    QCOMPARE(local3ecef1.R(z_id,x_id), 0);
    QVERIFY(local3ecef1.R(z_id,y_id) > 0);
    QVERIFY(local3ecef1.R(z_id,z_id) > 0);

    StereoVision::Geometry::AffineTransform<float> ecef2local1NWU = getLocalFrameAtPosNWU(lat, lon, alt);
    StereoVision::Geometry::AffineTransform<float> local3ecef1NWU  =
            StereoVision::Geometry::AffineTransform<float>(ecef2local1NWU .R.transpose(), -ecef2local1NWU .R.transpose()*ecef2local1NWU.t);

    QCOMPARE(local3ecef1NWU.R(z_id,y_id), 0);
    QVERIFY(local3ecef1NWU.R(z_id,x_id) > 0);
    QVERIFY(local3ecef1NWU.R(z_id,z_id) > 0);

    StereoVision::Geometry::AffineTransform<float> ecef2local1NED = getLocalFrameAtPosNED(lat, lon, alt);
    StereoVision::Geometry::AffineTransform<float> local3ecef1NED  =
            StereoVision::Geometry::AffineTransform<float>(ecef2local1NED .R.transpose(), -ecef2local1NED .R.transpose()*ecef2local1NED.t);

    QCOMPARE(local3ecef1NED.R(z_id,y_id), 0);
    QVERIFY(local3ecef1NED.R(z_id,x_id) > 0);
    QVERIFY(local3ecef1NED.R(z_id,z_id) < 0);

}

QTEST_MAIN(TestGeoTools)
#include "testGeoTools.moc"
