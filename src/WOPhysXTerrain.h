#pragma once

#include <functional>
#include <memory>

#include "WOGridECEFElevation.h"
#include "PhysXEngine.h"

namespace Aftr {
class WOPhysXTerrain : public WOGridECEFElevation {
public:
    WOMacroDeclaration(WOPhysXTerrain, WOGridECEFElevation);
    /*
    static WOPhysXTerrain* New(bool clip, const VectorD& upperLeft, const VectorD& lowerRight, unsigned int granularity, const VectorD& offset, const VectorD& scale, std::string elevationData, int splits = 2, float exageration = 0, bool useColors = false);
    static WOPhysXTerrain* New(const VectorD& upperLeft, const VectorD& lowerRight, unsigned int granularity, const VectorD& offset, const VectorD& scale, std::string elevationData, int splits = 2, float exageration = 0, bool useColors = false);
    static WOPhysXTerrain* New(const VectorD& upperLeft, const VectorD& lowerRight, unsigned int granularity, const VectorD& scale, std::string elevationData, int splits = 2, float exageration = 0, bool useColors = false, float oceanHeight = -1500);
    */

    virtual ~WOPhysXTerrain();

    // pull pose data from PhysX (calls updateCallback)
    virtual void pullFromPhysX();
    // push pose data to PhysX
    virtual void pushToPhysX() const;

    // have to overload all position/rotation updating methods to push those
    // changes to PhysX
    virtual void setPosition(const Vector& newXYZ);
    virtual void setPosition(float x, float y, float z);
    virtual void setPositionIgnoringAllChildren(const Vector& newXYZ);
    virtual void moveRelative(const Vector& dXdYdZ);
    virtual void moveRelativeIgnoringAllChildren(const Vector& dXdYdZ);

    virtual void rotateToIdentity();
    virtual void rotateAboutRelX(float deltaRadianAngle);
    virtual void rotateAboutRelY(float deltaRadianAngle);
    virtual void rotateAboutRelZ(float deltaRadianAngle);
    virtual void rotateAboutGlobalX(float deltaRadianAngle);
    virtual void rotateAboutGlobalY(float deltaRadianAngle);
    virtual void rotateAboutGlobalZ(float deltaRadianAngle);

    // set WO's PhysXEngine, thus creating its PhysX Actor and data
    void setPhysXEngine(const std::shared_ptr<PhysXEngine>& engine);
    // set callback for when WO receives a PhysX update
    void setPhysXUpdateCallback(const std::function<void()>& callback);

    // wrap calls of New directly to calls of New of WOGridECEFElevation
    template <class... Ts>
    static WOPhysXTerrain* New(Ts... args)
    {
        WOPhysXTerrain* wo = new WOPhysXTerrain();
        wo->onCreate(args...);
        return wo;
    }

protected:
    WOPhysXTerrain();

    std::shared_ptr<PhysXEngine> physxEngine;
    physx::PxRigidActor* physxActor;
    std::function<void()> updateCallback;

    void createPhysXActor();
};
}