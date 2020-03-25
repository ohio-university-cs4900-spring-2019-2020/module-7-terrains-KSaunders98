#include "GLViewTerrainModule.h"

#include <chrono>

#include "Axes.h" //We can set Axes to on/off with this
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "NetMessengerClient.h"
#include "NetMessengerServer.h"
#include "NetMessengerServerListener.h"
#include "NetMessengerServerSession.h"
#include "NetMessengerSessionContainer.h"
#include "PhysXEngine.h"
#include "WorldList.h" //This is where we place all of our WOs

//Different WO used by this module
#include "AftrGLRendererBase.h"
#include "Camera.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "CameraChaseActorSmooth.h"
#include "CameraStandard.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WO.h"
#include "WODynamicConvexMesh.h"
#include "WOLight.h"
#include "WOPhysXTerrain.h"
#include "WOSkyBox.h"

#include "NetMsgNewModel.h"
#include "NetMsgUpdateModel.h"

using namespace Aftr;
using namespace physx;

GLViewTerrainModule* GLViewTerrainModule::New(const std::vector<std::string>& args)
{
    GLViewTerrainModule* glv = new GLViewTerrainModule(args);
    glv->init(Aftr::GRAVITY, Vector(0, 0, -1.0f), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE);
    glv->onCreate();
    return glv;
}

GLViewTerrainModule::GLViewTerrainModule(const std::vector<std::string>& args)
    : GLView(args)
{
    //Initialize any member variables that need to be used inside of LoadMap() here.
    //Note: At this point, the Managers are not yet initialized. The Engine initialization
    //occurs immediately after this method returns (see GLViewTerrainModule::New() for
    //reference). Then the engine invoke's GLView::loadMap() for this module.
    //After loadMap() returns, GLView::onCreate is finally invoked.

    //The order of execution of a module startup:
    //GLView::New() is invoked:
    //    calls GLView::init()
    //       calls GLView::loadMap() (as well as initializing the engine's Managers)
    //    calls GLView::onCreate()

    //GLViewTerrainModule::onCreate() is invoked after this module's LoadMap() is completed.

    physxEngine = nullptr;
    netClient = nullptr;
}

void GLViewTerrainModule::onCreate()
{
    //GLViewTerrainModule::onCreate() is invoked after this module's LoadMap() is completed.
    //At this point, all the managers are initialized. That is, the engine is fully initialized.

    if (this->pe != NULL) {
        //optionally, change gravity direction and magnitude here
        //The user could load these values from the module's aftr.conf
        this->pe->setGravityNormalizedVector(Vector(0, 0, -1.0f));
        this->pe->setGravityScalar(Aftr::GRAVITY);
    }
    this->setActorChaseType(STANDARDEZNAV); //Default is STANDARDEZNAV mode
    getCamera()->setCameraAxisOfHorizontalRotationViaMouseMotion(terrainNormal);
    getCamera()->setPosition(terrainNormal * 500.0f);
    //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
}

GLViewTerrainModule::~GLViewTerrainModule()
{
    //Implicitly calls GLView::~GLView()
    if (physxEngine != nullptr)
        physxEngine->shutdown();
}

void GLViewTerrainModule::updateWorld()
{
    GLView::updateWorld(); //Just call the parent's update world first.
        //If you want to add additional functionality, do it after
        //this call.

    if (physxEngine != nullptr) {
        using namespace std::chrono;

        // calculate delta time
        static auto last_time = steady_clock::now();
        auto now = steady_clock::now();
        float dt = duration_cast<duration<float>>(now - last_time).count();
        last_time = now;

        physxEngine->updateSimulation(dt);
    }
}

void GLViewTerrainModule::onResizeWindow(GLsizei width, GLsizei height)
{
    GLView::onResizeWindow(width, height); //call parent's resize method.
}

void GLViewTerrainModule::onMouseDown(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseDown(e);
}

void GLViewTerrainModule::onMouseDownSelection(unsigned int x, unsigned int y, Camera& cam)
{
    GLView::onMouseDownSelection(x, y, cam);

    if (getLastSelectedCoordinate() != nullptr) {
        Vector pos = *getLastSelectedCoordinate() + terrainNormal * 50.0f;

        spawnNewModel(teapotPath, Vector(2, 2, 2), pos);
    }
}

void GLViewTerrainModule::onMouseUp(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseUp(e);
}

void GLViewTerrainModule::onMouseMove(const SDL_MouseMotionEvent& e)
{
    GLView::onMouseMove(e);
}

void GLViewTerrainModule::onKeyDown(const SDL_KeyboardEvent& key)
{
    GLView::onKeyDown(key);
    if (key.keysym.sym == SDLK_0)
        this->setNumPhysicsStepsPerRender(1);

    if (key.keysym.sym == SDLK_1) {
    }
}

void GLViewTerrainModule::onKeyUp(const SDL_KeyboardEvent& key)
{
    GLView::onKeyUp(key);
}

void Aftr::GLViewTerrainModule::loadMap()
{
    this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
    this->actorLst = new WorldList();
    this->netLst = new WorldList();

    ManagerOpenGLState::GL_CLIPPING_PLANE = 10000.0;
    ManagerOpenGLState::GL_NEAR_PLANE = 0.5f;
    ManagerOpenGLState::enableFrustumCulling = false;
    Axes::isVisible = false;
    this->glRenderer->isUsingShadowMapping(false); //set to TRUE to enable shadow mapping, must be using GL 3.2+

    this->cam->setPosition(50, 50, 50);
    this->cam->setCameraLookAtPoint(Vector(0, 0, 0));

    std::string mountainPath(ManagerEnvironmentConfiguration::getLMM() + "/models/mountain.obj");
    teapotPath = ManagerEnvironmentConfiguration::getLMM() + "/models/teapot.obj";

    std::string port = ManagerEnvironmentConfiguration::getVariableValue("NetServerListenPort");
    if (port == "12683") {
        physxEngine = std::make_shared<PhysXEngine>();
        netClient = std::shared_ptr<NetMessengerClient>(NetMessengerClient::New("127.0.0.1", "12682"));
    } else {
        netClient = std::shared_ptr<NetMessengerClient>(NetMessengerClient::New("127.0.0.1", "12683"));
    }

    //SkyBox Textures readily available
    std::vector<std::string> skyBoxImageNames; //vector to store texture paths
    skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg");

    float ga = 0.1f; //Global Ambient Light level for this module
    ManagerLight::setGlobalAmbientLight(aftrColor4f(ga, ga, ga, 1.0f));
    WOLight* light = WOLight::New();
    light->isDirectionalLight(true);
    light->setPosition(Vector(0, 0, 100));
    //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
    //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
    light->getModel()->setDisplayMatrix(Mat4::rotateIdentityMat({ 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD));
    light->setLabel("Light");
    worldLst->push_back(light);

    //Create the SkyBox
    WO* wo = WOSkyBox::New(skyBoxImageNames.at(0), this->getCameraPtrPtr());
    wo->setPosition(Vector(0, 0, 0));
    wo->setLabel("Sky Box");
    wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(wo);

    float top = AftrUtilities::toDouble(ManagerEnvironmentConfiguration::getVariableValue("Top"));
    float bottom = AftrUtilities::toDouble(ManagerEnvironmentConfiguration::getVariableValue("Bottom"));

    float left = AftrUtilities::toDouble(ManagerEnvironmentConfiguration::getVariableValue("Left"));
    float right = AftrUtilities::toDouble(ManagerEnvironmentConfiguration::getVariableValue("Right"));

    float vert = top - bottom;
    float horz = right - left;

    VectorD offset((top + bottom) / 2, (left + right) / 2, 0);
    Vector centerOfWorld = offset.toVecS().toECEFfromWGS84();
    terrainNormal = centerOfWorld.normalizeMe();

    if (physxEngine != nullptr) {
        // set gravity along terrain normal
        physxEngine->setGravity(terrainNormal * -Aftr::GRAVITY);
    }

    VectorD scale = VectorD(1.0f, 1.0f, 1.0f);
    VectorD upperLeft(top, left, 0);
    VectorD lowerRight(bottom, right, 0);

    // load elevation data as a grid with a PhysX actor
    std::string elev = ManagerEnvironmentConfiguration::getLMM() + "/images/" + ManagerEnvironmentConfiguration::getVariableValue("Elevation");
    WOPhysXTerrain* grid = WOPhysXTerrain::New(upperLeft, lowerRight, 0, offset, scale, elev, 2, 0, false);
    grid->setPosition(0, 0, -500);
    grid->setLabel("grid");
    worldLst->push_back(grid);
    if (physxEngine != nullptr) {
        grid->setPhysXEngine(physxEngine);
    }

    // load and set texture to grid
    Texture* tex = ManagerTexture::loadTexture(ManagerEnvironmentConfiguration::getLMM() + "/images/" + ManagerEnvironmentConfiguration::getVariableValue("Imagery"));
    for (size_t i = 0; i < grid->getModel()->getModelDataShared()->getModelMeshes().size(); i++)
        grid->getModel()->getModelDataShared()->getModelMeshes().at(i)->getSkin().getMultiTextureSet().at(0) = tex;
    grid->getModel()->isUsingBlending(false);
}

void GLViewTerrainModule::spawnNewModel(const std::string& path, const Vector& scale, const Vector& position, bool sendMsg)
{
    WOPhysXActor* model = WODynamicConvexMesh::New(path, scale, MESH_SHADING_TYPE::mstFLAT);
    model->setPosition(position);
    model->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    worldLst->push_back(model);
    models.push_back(model);

    if (sendMsg) {
        // send msg to other instance
        NetMsgNewModel msg;
        msg.path = path;
        msg.scale = scale;
        msg.position = position;
        netClient->sendNetMsgSynchronousTCP(msg);
    }

    if (physxEngine != nullptr) {
        unsigned int id = static_cast<unsigned int>(models.size()) - 1;

        // setup model's physics
        model->setPhysXEngine(physxEngine);

        model->setPhysXUpdateCallback([this, id, model]() {
            // send update message to other instance
            NetMsgUpdateModel msg;
            msg.id = id;
            msg.displayMatrix = model->getDisplayMatrix();
            msg.position = model->getPosition();
            netClient->sendNetMsgSynchronousTCP(msg);
        });
    }
}

void GLViewTerrainModule::updateModel(unsigned int id, const Mat4& displayMatrix, const Vector& position)
{
    models.at(id)->getModel()->setDisplayMatrix(displayMatrix);
    models.at(id)->setPosition(position);
}
