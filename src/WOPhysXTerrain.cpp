#include "WOPhysXTerrain.h"

#include "Mat4.h"
#include "Model.h"

using namespace Aftr;
using namespace physx;

/*WOPhysXTerrain* WOPhysXTerrain::New(bool clip, const VectorD& upperLeft, const VectorD& lowerRight, unsigned int granularity, const VectorD& offset, const VectorD& scale, std::string elevationData, int splits = 2, float exageration = 0, bool useColors = false)
{
    WOPhysXTerrain* wo = new WOPhysXTerrain();
    wo->WOGridECEFElevation::onCreate(clip, upperLeft, lowerRight, granularity, offset, scale, elevationData, splits, exageration, useColors);
    wo->WOStaticTriangleMesh::onCreate()
    return wo;
}

WOPhysXTerrain* WOPhysXTerrain::New(const VectorD& upperLeft, const VectorD& lowerRight, unsigned int granularity, const VectorD& offset, const VectorD& scale, std::string elevationData, int splits = 2, float exageration = 0, bool useColors = false)
{
}

WOPhysXTerrain* WOPhysXTerrain::New(const VectorD& upperLeft, const VectorD& lowerRight, unsigned int granularity, const VectorD& scale, std::string elevationData, int splits = 2, float exageration = 0, bool useColors = false, float oceanHeight = -1500)
{
}

WOPhysXTerrain* WOPhysXTerrain::New(const std::string& modelFileName, Vector scale, MESH_SHADING_TYPE shadingType)
{
    WOPhysXTerrain* wo = new WOPhysXTerrain();
    wo->WOGridECEFElevation::onCreate(modelFileName, scale, shadingType);
    return wo;
}*/

WOPhysXTerrain::WOPhysXTerrain()
    : IFace(this)
    , WOGridECEFElevation()
{
    physxEngine = nullptr;
    physxActor = nullptr;
    updateCallback = nullptr;
}

WOPhysXTerrain::~WOPhysXTerrain()
{
    if (physxEngine != nullptr && physxActor != nullptr)
        physxEngine->destroyActor(physxActor);
}

void WOPhysXTerrain::pullFromPhysX()
{
    PxTransform t = physxActor->getGlobalPose();
    PxMat44 m = PxMat44(t);
    PxVec3 p = t.p;

    Mat4 mat;
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            mat[i * 4 + j] = m[i][j];
        }
    }
    getModel()->setDisplayMatrix(mat);
    setPosition(p.x, p.y, p.z);

    if (updateCallback != nullptr)
        updateCallback();
}

void WOPhysXTerrain::pushToPhysX() const
{
    if (physxActor != nullptr) {
        Mat4 mat = getDisplayMatrix();
        Vector p = getPosition();

        PxMat44 m;
        for (unsigned int i = 0; i < 3; ++i) {
            for (unsigned int j = 0; j < 3; ++j) {
                m[i][j] = mat[i * 4 + j];
            }
        }
        m[3] = PxVec4(p.x, p.y, p.z, 1.0f);
        physxActor->setGlobalPose(PxTransform(m));
    }
}

void WOPhysXTerrain::setPosition(const Vector& newXYZ)
{
    WO::setPosition(newXYZ);
    pushToPhysX();
}

void WOPhysXTerrain::setPosition(float x, float y, float z)
{
    WO::setPosition(x, y, z);
    pushToPhysX();
}

void WOPhysXTerrain::setPositionIgnoringAllChildren(const Vector& newXYZ)
{
    WO::setPositionIgnoringAllChildren(newXYZ);
    pushToPhysX();
}

void WOPhysXTerrain::moveRelative(const Vector& dXdYdZ)
{
    WO::moveRelative(dXdYdZ);
    pushToPhysX();
}

void WOPhysXTerrain::moveRelativeIgnoringAllChildren(const Vector& dXdYdZ)
{
    WO::moveRelativeIgnoringAllChildren(dXdYdZ);
    pushToPhysX();
}

void WOPhysXTerrain::rotateToIdentity()
{
    WO::rotateToIdentity();
    pushToPhysX();
}

void WOPhysXTerrain::rotateAboutRelX(float deltaRadianAngle)
{
    WO::rotateAboutRelX(deltaRadianAngle);
    pushToPhysX();
}

void WOPhysXTerrain::rotateAboutRelY(float deltaRadianAngle)
{
    WO::rotateAboutRelY(deltaRadianAngle);
    pushToPhysX();
}

void WOPhysXTerrain::rotateAboutRelZ(float deltaRadianAngle)
{
    WO::rotateAboutRelZ(deltaRadianAngle);
    pushToPhysX();
}

void WOPhysXTerrain::rotateAboutGlobalX(float deltaRadianAngle)
{
    WO::rotateAboutGlobalX(deltaRadianAngle);
    pushToPhysX();
}

void WOPhysXTerrain::rotateAboutGlobalY(float deltaRadianAngle)
{
    WO::rotateAboutGlobalY(deltaRadianAngle);
    pushToPhysX();
}

void WOPhysXTerrain::rotateAboutGlobalZ(float deltaRadianAngle)
{
    WO::rotateAboutGlobalZ(deltaRadianAngle);
    pushToPhysX();
}

void WOPhysXTerrain::setPhysXEngine(const std::shared_ptr<PhysXEngine>& engine)
{
    physxEngine = engine;
    createPhysXActor();
}

void WOPhysXTerrain::setPhysXUpdateCallback(const std::function<void()>& callback)
{
    updateCallback = callback;
}

void WOPhysXTerrain::createPhysXActor()
{
    physxActor = physxEngine->createTriangleMesh(getModel());
    physxActor->userData = this;
    pushToPhysX();
}