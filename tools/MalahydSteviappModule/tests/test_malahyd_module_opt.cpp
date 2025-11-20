#include <QTest>

#include <StereoVision/geometry/rotations.h>

#include <steviapp/datablocks/project.h>
#include <steviapp/datablocks/image.h>
#include <steviapp/datablocks/camera.h>
#include <steviapp/datablocks/cameras/pushbroompinholecamera.h>
#include <steviapp/datablocks/landmark.h>
#include <steviapp/datablocks/correspondencesset.h>
#include <steviapp/datablocks/localcoordinatesystem.h>
#include <steviapp/datablocks/trajectory.h>
#include <steviapp/datablocks/mounting.h>

#include <steviapp/sparsesolver/modularsbasolver.h>
#include <steviapp/sparsesolver/sbamodules/landmarkssbamodule.h>
#include <steviapp/sparsesolver/sbamodules/pinholecameraprojectormodule.h>
#include <steviapp/sparsesolver/sbamodules/imagealignementsbamodule.h>
#include <steviapp/sparsesolver/sbamodules/correspondencessetsbamodule.h>
#include <steviapp/sparsesolver/sbamodules/localcoordinatesystemsbamodule.h>
#include <steviapp/sparsesolver/sbamodules/trajectorybasesbamodule.h>
#include <steviapp/sparsesolver/sbamodules/mountingssbamodule.h>

#include <steviapp/testutils/datablocks/generatedtrajectory.h>

#include "../datablocks/bilacquisitiondata.h"
#include "../solving/bilsequencesbamodule.h"

class TestMalahydModule : public QObject {

    Q_OBJECT
private Q_SLOTS:

    void initTestCase();

    void billSequence2ImageCorresps();
    void billSequence2billSequenceCorresps();
};

void TestMalahydModule::initTestCase() {

    srand(time(nullptr));

    //configure libraries
    google::InitGoogleLogging("TestSBASolver");

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    pF.addType(new StereoVisionApp::LandmarkFactory(this));
    pF.addType(new StereoVisionApp::ImageFactory(this));
    pF.addType(new StereoVisionApp::CameraFactory(this));
    pF.addType(new StereoVisionApp::CorrespondencesSetFactory(this));
    pF.addType(new StereoVisionApp::LocalCoordinateSystemFactory(this));
    pF.addType(new StereoVisionApp::GeneratedTrajectoryFactory(this)); //ensure the trajectores can be configured with generators
    pF.addType(new StereoVisionApp::MountingFactory(this));
    pF.addType(new PikaLTools::BilSequenceAcquisitionDataFactory(this));
    pF.addType(new StereoVisionApp::PushBroomPinholeCameraFactory(this));

}

void TestMalahydModule::billSequence2ImageCorresps() {
    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 imageId = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());
    qint64 cameraId = project.createDataBlock(StereoVisionApp::Camera::staticMetaObject.className());

    qint64 bilSequenceId = project.createDataBlock(PikaLTools::BilSequenceAcquisitionData::staticMetaObject.className());
    qint64 pushBroomId = project.createDataBlock(StereoVisionApp::PushBroomPinholeCamera::staticMetaObject.className());
    qint64 mountingId = project.createDataBlock(StereoVisionApp::Mounting::staticMetaObject.className());
    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());

    StereoVisionApp::Image* img = project.getDataBlock<StereoVisionApp::Image>(imageId);
    StereoVisionApp::Camera* cam = project.getDataBlock<StereoVisionApp::Camera>(cameraId);

    PikaLTools::BilSequenceAcquisitionData* bilSeq = project.getDataBlock<PikaLTools::BilSequenceAcquisitionData>(bilSequenceId);
    StereoVisionApp::PushBroomPinholeCamera* pushBroom = project.getDataBlock<StereoVisionApp::PushBroomPinholeCamera>(pushBroomId);
    StereoVisionApp::Mounting* mounting = project.getDataBlock<StereoVisionApp::Mounting>(mountingId);
    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    QVERIFY(img != nullptr);
    QVERIFY(cam != nullptr);
    QVERIFY(bilSeq != nullptr);
    QVERIFY(pushBroom != nullptr);
    QVERIFY(mounting != nullptr);
    QVERIFY(traj != nullptr);
    QVERIFY(correspSet != nullptr);

    StereoVisionApp::GeneratedTrajectory* genTraj = qobject_cast<StereoVisionApp::GeneratedTrajectory*>(traj);

    QVERIFY(genTraj != nullptr);

    img->assignCamera(cameraId);

    bilSeq->assignCamera(pushBroomId);
    bilSeq->assignMounting(mountingId);
    bilSeq->assignTrajectory(trajectoryId);

    cam->setImHeight(480);
    cam->setImWidth(640);

    pushBroom->setImWidth(1201);

    StereoVisionApp::floatParameter fLen(35);
    StereoVisionApp::floatParameter ppX(pFloatType(cam->imWidth())/2);
    StereoVisionApp::floatParameter ppY(pFloatType(cam->imHeight())/2);

    StereoVisionApp::floatParameter pushBroomFLen(35);
    StereoVisionApp::floatParameter pushBroomPpX(pFloatType(pushBroom->imWidth())/2);

    StereoVisionApp::floatParameter zero(0);

    cam->setFLen(fLen);
    cam->setOpticalCenterX(ppX);
    cam->setOpticalCenterY(ppY);

    cam->setB1(zero);
    cam->setB2(zero);

    cam->setP1(zero);
    cam->setP2(zero);

    cam->setK1(zero);
    cam->setK2(zero);
    cam->setK3(zero);
    cam->setK4(zero);
    cam->setK5(zero);
    cam->setK6(zero);

    pushBroom->setFLen(pushBroomFLen);
    pushBroom->setOpticalCenterX(pushBroomPpX);

    pushBroom->setA0(zero);
    pushBroom->setA1(zero);
    pushBroom->setA2(zero);
    pushBroom->setA3(zero);
    pushBroom->setA4(zero);
    pushBroom->setA5(zero);

    pushBroom->setB0(zero);
    pushBroom->setB1(zero);
    pushBroom->setB2(zero);
    pushBroom->setB3(zero);
    pushBroom->setB4(zero);
    pushBroom->setB5(zero);

    StereoVision::Geometry::RigidBodyTransform<double> img2world(Eigen::Vector3d::Zero(), Eigen::Vector3d(0,0,-10)); //cam2world
    StereoVision::Geometry::RigidBodyTransform<double> world2img = img2world.inverse();

    img->setXCoord(img2world.t.x());
    img->setYCoord(img2world.t.y());
    img->setZCoord(img2world.t.z());

    img->setXRot(img2world.r.x());
    img->setYRot(img2world.r.y());
    img->setZRot(img2world.r.z());

    StereoVision::Geometry::RigidBodyTransform<double> pushbroom2Body(Eigen::Vector3d(0.01,-0.05,0.02), Eigen::Vector3d(0.3,0.2,-0.1));
    //StereoVision::Geometry::RigidBodyTransform<double> pushbroom2Body(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
    StereoVision::Geometry::RigidBodyTransform<double> body2PushBroom = pushbroom2Body.inverse();

    mounting->setXRot(StereoVisionApp::floatParameter(body2PushBroom.r.x()));
    mounting->setYRot(StereoVisionApp::floatParameter(body2PushBroom.r.y()));
    mounting->setZRot(StereoVisionApp::floatParameter(body2PushBroom.r.z()));

    mounting->setXCoord(StereoVisionApp::floatParameter(body2PushBroom.t.x()));
    mounting->setYCoord(StereoVisionApp::floatParameter(body2PushBroom.t.y()));
    mounting->setZCoord(StereoVisionApp::floatParameter(body2PushBroom.t.z()));

    using TrajGeneratorInfos = StereoVisionApp::GeneratedTrajectory::TrajGeneratorInfos;

    constexpr double t0 = 0;
    constexpr double tf = 100;
    constexpr double dt = tf-t0;

    constexpr double y0 = -50;
    constexpr double yf = 50;

    double dtPos = 0.5;
    double dtAcc = 0.1;

    struct PositionCalculator {
        static Eigen::Vector3d pos(double t) {
            double x = 0;
            double y = (yf-y0)*t/dt + y0;
            double z = 0;
            return Eigen::Vector3d(x,y,z);
        }
    };

    genTraj->setInitialAndFinalTimes(t0,tf);

    genTraj->setAngularSpeedGenerator(TrajGeneratorInfos{[] (double t) {
                                                             (void) t; return Eigen::Vector3d(0,0,0);
                                                         }, dtAcc}); //constant rotation rate
    genTraj->setAccelerationGenerator(TrajGeneratorInfos{[] (double t) {
                                                             (void) t; return Eigen::Vector3d(0,0,0);
                                                         }, dtAcc});
    genTraj->setPositionGenerator(TrajGeneratorInfos{[] (double t) {
                                                         return PositionCalculator::pos(t);
                                                     }, dtPos});
    genTraj->setOrientationGenerator(TrajGeneratorInfos{[] (double t) {
                                                            (void) t; return Eigen::Vector3d(0,0,0);
                                                        }, dtPos});

    traj->setPreIntegrationTime(0.5);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);

    traj->setFixed(true);

    //Points
    std::vector<Eigen::Vector3d> points =
        {
            Eigen::Vector3d{42,0.05*dt,33},
            Eigen::Vector3d{33,0.2*dt,50},
            Eigen::Vector3d{-6,0.3*dt,70},
            Eigen::Vector3d{-49,0.6*dt,42},
            Eigen::Vector3d{-69,0.9*dt,33}
        };

    std::vector<Eigen::Vector3d> pointsWorld;
    pointsWorld.reserve(points.size());
    std::vector<Eigen::Vector3d> pointsImg;
    pointsImg.reserve(points.size());

    for (Eigen::Vector3d const& point : points) {

        constexpr StereoVisionApp::Correspondences::Types UV = StereoVisionApp::Correspondences::UV;
        constexpr StereoVisionApp::Correspondences::Types UVT = StereoVisionApp::Correspondences::UVT;

        using UVCorresp = StereoVisionApp::Correspondences::Typed<UV>;
        using UVTCorresp = StereoVisionApp::Correspondences::Typed<UVT>;

        double t = point.y();

        Eigen::Vector3d pointPushBroom = point;
        pointPushBroom.y() = 0;

        StereoVision::Geometry::RigidBodyTransform<double> body2World(Eigen::Vector3d::Zero(),
                                                                      PositionCalculator::pos(t));

        Eigen::Vector3d pointWorld = body2World*pushbroom2Body*pointPushBroom;
        pointsWorld.push_back(pointWorld);

        Eigen::Vector3d ptImg = world2img*pointWorld;
        pointsImg.push_back(ptImg);

        Eigen::Vector2d uvImg = ptImg.block<2,1>(0,0)/ptImg.z() * fLen.value();
        uvImg.x() += ppX.value();
        uvImg.y() += ppY.value();

        Eigen::Vector2d uvPushBroom = pointPushBroom.block<2,1>(0,0)/pointPushBroom.z() * pushBroomFLen.value();
        uvPushBroom.x() += pushBroomPpX.value();

        UVCorresp uvCorresp;
        uvCorresp.blockId = imageId;
        uvCorresp.u = uvImg.x();
        uvCorresp.v = uvImg.y();
        uvCorresp.sigmaU = 1;
        uvCorresp.sigmaV = 1;

        UVTCorresp uvtCorresp;
        uvtCorresp.blockId = bilSequenceId;
        uvtCorresp.u = uvPushBroom.x();
        uvtCorresp.v = uvPushBroom.y();
        uvtCorresp.t = t;
        uvtCorresp.sigmaU = 1;
        uvtCorresp.sigmaV = 1;

        correspSet->addCorrespondence({uvtCorresp, uvCorresp});

    }

    StereoVisionApp::ModularSBASolver sbaSolver(&project);
    sbaSolver.setSilent(true);


    StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
        new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
    bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);

    bool lmModuleAdded = sbaSolver.addModule(new StereoVisionApp::LandmarksSBAModule());
    bool imModuleAdded = sbaSolver.addModule(new StereoVisionApp::ImageAlignementSBAModule());
    bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());
    bool bilModuleAdded = sbaSolver.addModule(new PikaLTools::BilSequenceSBAModule());

    QVERIFY(trajModuleAdded);
    QVERIFY(lmModuleAdded);
    QVERIFY(imModuleAdded);
    QVERIFY(correspModuleAdded);
    QVERIFY(bilModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(imageId));
    QVERIFY(sbaSolver.itemIsObservable(bilSequenceId));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    qInfo() << "Measured cost: " << cost;
    QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution
}


void TestMalahydModule::billSequence2billSequenceCorresps() {
    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 pushBroomId = project.createDataBlock(StereoVisionApp::PushBroomPinholeCamera::staticMetaObject.className());
    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    qint64 bilSequence1Id = project.createDataBlock(PikaLTools::BilSequenceAcquisitionData::staticMetaObject.className());
    qint64 mounting1Id = project.createDataBlock(StereoVisionApp::Mounting::staticMetaObject.className());

    qint64 bilSequence2Id = project.createDataBlock(PikaLTools::BilSequenceAcquisitionData::staticMetaObject.className());
    qint64 mounting2Id = project.createDataBlock(StereoVisionApp::Mounting::staticMetaObject.className());

    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());

    StereoVisionApp::PushBroomPinholeCamera* pushBroom = project.getDataBlock<StereoVisionApp::PushBroomPinholeCamera>(pushBroomId);
    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    PikaLTools::BilSequenceAcquisitionData* bilSeq1 = project.getDataBlock<PikaLTools::BilSequenceAcquisitionData>(bilSequence1Id);
    StereoVisionApp::Mounting* mounting1 = project.getDataBlock<StereoVisionApp::Mounting>(mounting1Id);

    PikaLTools::BilSequenceAcquisitionData* bilSeq2 = project.getDataBlock<PikaLTools::BilSequenceAcquisitionData>(bilSequence2Id);
    StereoVisionApp::Mounting* mounting2 = project.getDataBlock<StereoVisionApp::Mounting>(mounting2Id);

    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    QVERIFY(pushBroom != nullptr);
    QVERIFY(traj != nullptr);

    QVERIFY(bilSeq1 != nullptr);
    QVERIFY(mounting1 != nullptr);

    QVERIFY(bilSeq2 != nullptr);
    QVERIFY(mounting2 != nullptr);

    QVERIFY(correspSet != nullptr);

    StereoVisionApp::GeneratedTrajectory* genTraj = qobject_cast<StereoVisionApp::GeneratedTrajectory*>(traj);

    QVERIFY(genTraj != nullptr);

    StereoVisionApp::floatParameter fLen(35);
    StereoVisionApp::floatParameter ppX(pFloatType(pushBroom->imWidth())/2);

    StereoVisionApp::floatParameter zero(0);

    pushBroom->setFLen(fLen);
    pushBroom->setOpticalCenterX(ppX);

    pushBroom->setA0(zero);
    pushBroom->setA1(zero);
    pushBroom->setA2(zero);
    pushBroom->setA3(zero);
    pushBroom->setA4(zero);
    pushBroom->setA5(zero);

    pushBroom->setB0(zero);
    pushBroom->setB1(zero);
    pushBroom->setB2(zero);
    pushBroom->setB3(zero);
    pushBroom->setB4(zero);
    pushBroom->setB5(zero);

    StereoVision::Geometry::RigidBodyTransform<double> pushbroom12Body(Eigen::Vector3d(0,0,0), Eigen::Vector3d(-10,0,0));
    StereoVision::Geometry::RigidBodyTransform<double> body2PushBroom1 = pushbroom12Body.inverse();

    mounting1->setXRot(StereoVisionApp::floatParameter(body2PushBroom1.r.x()));
    mounting1->setYRot(StereoVisionApp::floatParameter(body2PushBroom1.r.y()));
    mounting1->setZRot(StereoVisionApp::floatParameter(body2PushBroom1.r.z()));

    mounting1->setXCoord(StereoVisionApp::floatParameter(body2PushBroom1.t.x()));
    mounting1->setYCoord(StereoVisionApp::floatParameter(body2PushBroom1.t.y()));
    mounting1->setZCoord(StereoVisionApp::floatParameter(body2PushBroom1.t.z()));

    StereoVision::Geometry::RigidBodyTransform<double> pushbroom22Body(Eigen::Vector3d(0,0,0), Eigen::Vector3d(10,0,0));
    StereoVision::Geometry::RigidBodyTransform<double> body2PushBroom2 = pushbroom12Body.inverse();

    mounting2->setXRot(StereoVisionApp::floatParameter(body2PushBroom2.r.x()));
    mounting2->setYRot(StereoVisionApp::floatParameter(body2PushBroom2.r.y()));
    mounting2->setZRot(StereoVisionApp::floatParameter(body2PushBroom2.r.z()));

    mounting2->setXCoord(StereoVisionApp::floatParameter(body2PushBroom2.t.x()));
    mounting2->setYCoord(StereoVisionApp::floatParameter(body2PushBroom2.t.y()));
    mounting2->setZCoord(StereoVisionApp::floatParameter(body2PushBroom2.t.z()));

    using TrajGeneratorInfos = StereoVisionApp::GeneratedTrajectory::TrajGeneratorInfos;

    constexpr double t0 = 0;
    constexpr double tf = 100;
    constexpr double dt = tf-t0;

    constexpr double y0 = -50;
    constexpr double yf = 50;

    double dtPos = 0.5;
    double dtAcc = 0.1;

    struct PositionCalculator {
        static Eigen::Vector3d pos(double t) {
            double x = 0;
            double y = (yf-y0)*t/dt + y0;
            double z = 0;
            return Eigen::Vector3d(x,y,z);
        }
    };

    genTraj->setInitialAndFinalTimes(t0,tf);

    genTraj->setAngularSpeedGenerator(TrajGeneratorInfos{[] (double t) {
                                                             (void) t; return Eigen::Vector3d(0,0,0);
                                                         }, dtAcc}); //constant rotation rate
    genTraj->setAccelerationGenerator(TrajGeneratorInfos{[] (double t) {
                                                             (void) t; return Eigen::Vector3d(0,0,0);
                                                         }, dtAcc});
    genTraj->setPositionGenerator(TrajGeneratorInfos{[] (double t) {
                                                         return PositionCalculator::pos(t);
                                                     }, dtPos});
    genTraj->setOrientationGenerator(TrajGeneratorInfos{[] (double t) {
                                                            (void) t; return Eigen::Vector3d(0,0,0);
                                                        }, dtPos});
    traj->setPreIntegrationTime(0.5);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);

    traj->setFixed(true);

    //Points
    std::vector<Eigen::Vector3d> points =
        {
            Eigen::Vector3d{42,0.05*dt,33},
            Eigen::Vector3d{33,0.2*dt,50},
            Eigen::Vector3d{-6,0.3*dt,70},
            Eigen::Vector3d{-49,0.6*dt,42},
            Eigen::Vector3d{-69,0.9*dt,33}
        };

    for (Eigen::Vector3d const& point : points) {

        constexpr StereoVisionApp::Correspondences::Types UVT = StereoVisionApp::Correspondences::UVT;

        using UVTCorresp = StereoVisionApp::Correspondences::Typed<UVT>;

        double t = point.y();

        StereoVision::Geometry::RigidBodyTransform<double> body2World(Eigen::Vector3d::Zero(),
                                                                      PositionCalculator::pos(t));

        Eigen::Vector3d pointWorld = point;
        pointWorld.y() = body2World.t.y();

        StereoVision::Geometry::RigidBodyTransform<double> world2body = body2World.inverse();

        Eigen::Vector3d ptLocal = world2body*pointWorld;

        Eigen::Vector3d pointPushBroom1 = body2PushBroom2*ptLocal;
        Eigen::Vector3d pointPushBroom2 = body2PushBroom2*ptLocal;

        Eigen::Vector2d uvPushBroom1 = pointPushBroom1.block<2,1>(0,0)/pointPushBroom1.z() * fLen.value();
        uvPushBroom1.x() += ppX.value();

        Eigen::Vector2d uvPushBroom2 = pointPushBroom2.block<2,1>(0,0)/pointPushBroom2.z() * fLen.value();
        uvPushBroom2.x() += ppX.value();

        QVERIFY(std::abs(pointPushBroom1.y()) < 1e-3);
        QVERIFY(std::abs(pointPushBroom2.y()) < 1e-3);

        UVTCorresp uvtCorresp1;
        uvtCorresp1.blockId = bilSequence1Id;
        uvtCorresp1.u = uvPushBroom2.x();
        uvtCorresp1.v = uvPushBroom2.y();
        uvtCorresp1.t = t;
        uvtCorresp1.sigmaU = 1;
        uvtCorresp1.sigmaV = 1;

        UVTCorresp uvtCorresp2;
        uvtCorresp2.blockId = bilSequence2Id;
        uvtCorresp2.u = uvPushBroom2.x();
        uvtCorresp2.v = uvPushBroom2.y();
        uvtCorresp2.t = t;
        uvtCorresp2.sigmaU = 1;
        uvtCorresp2.sigmaV = 1;

        correspSet->addCorrespondence({uvtCorresp1, uvtCorresp2});
    }

    StereoVisionApp::ModularSBASolver sbaSolver(&project);
    sbaSolver.setSilent(true);


    StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
        new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
    bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);

    bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());
    bool bilModuleAdded = sbaSolver.addModule(new PikaLTools::BilSequenceSBAModule());

    QVERIFY(trajModuleAdded);
    QVERIFY(correspModuleAdded);
    QVERIFY(bilModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(bilSequence1Id));
    QVERIFY(sbaSolver.itemIsObservable(bilSequence2Id));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    qInfo() << "Measured cost: " << cost;
    QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution
}

QTEST_MAIN(TestMalahydModule)
#include "test_malahyd_module_opt.moc"
