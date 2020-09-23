//
// Copyright (c) 2008-2018 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
/*

    Written by Ajay Bhaga 2020

*/
#include <sstream>
#include <string>
#include <iostream>
#include <Urho3D/Audio/Audio.h>
#include <Urho3D/UI/Button.h>
#include <Urho3D/Urho2D/CollisionBox2D.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/MemoryBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/Animation.h>
#include <Urho3D/Graphics/AnimationState.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/BillboardSet.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/GraphicsEvents.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/Audio/SoundSource3D.h>
#include <Urho3D/Audio/Sound.h>
#include <Urho3D/Audio/SoundListener.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Terrain.h>
#include <Urho3D/Graphics/TerrainPatch.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Input/Controls.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Network/Network.h>
#include <Urho3D/Network/Connection.h>
#include <Urho3D/Network/NetworkEvents.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/UI/UIEvents.h>


#include <Urho3D/IO/IOEvents.h>
#include <Urho3D/IO/VectorBuffer.h>

#include <Urho3D/Urho2D/AnimatedSprite2D.h>
#include <Urho3D/Urho2D/AnimationSet2D.h>
#include <Urho3D/UI/BorderImage.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/Urho2D/ParticleEffect2D.h>
#include <Urho3D/Urho2D/ParticleEmitter2D.h>
#include <Urho3D/Urho2D/RigidBody2D.h>
#include <Urho3D/Core/StringUtils.h>
#include <Urho3D/Graphics/Texture2D.h>
#include <Urho3D/Urho2D/TileMap2D.h>
#include <Urho3D/Urho2D/TileMap3D.h>
#include <Urho3D/Urho2D/TmxFile2D.h>
#include <Urho3D/Scene/ValueAnimation.h>

#include "network/Server.h"
#include "network/ClientObj.h"
#include "network/NetworkActor.h"


//#include <Urho3D/Physics/RaycastVehicle.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>

#include <Urho3D/Urho2D/PhysicsEvents2D.h>
#include <Urho3D/Urho2D/PhysicsWorld2D.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Urho2D/RigidBody2D.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/Core/StringUtils.h>
#include <Urho3D/Urho2D/TileMap2D.h>
//#include <Urho3D/Urho2D/TileMapLayer2D.h>
#include <Urho3D/Urho2D/TileMap3D.h>
//#include <Urho3D/Urho2D/TileMapLayer3D.h>
#include <Urho3D/Urho2D/TmxFile2D.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/UI/UIEvents.h>

#include <Urho3D/Urho2D/ParticleEffect2D.h>
#include <Urho3D/Urho2D/ParticleEmitter2D.h>


#include <Urho3D/DebugNew.h>

#include "GameController.h"
#include "Character2D.h"
#include "Object2D.h"
#include "Utilities2D/Mover.h"
#include "MayaScape.h"
#include <MayaScape/ai/evolution_manager.h>
#include <MayaScape/network/CSP_Server.h>
#include "RaycastVehicle.h"
#include "MayaScape/Vehicle.h"
#include "WheelTrackModel.h"
#include "SmoothStep.h"


// AgentSim shared libs
#include "shared_libs.h"
#include "types.h"
#include "Player.h"
#include "Bullet.h"
#include "AP.h"

#include "boids.h"
#include "../../Urho3D/Input/Controls.h"


//#define CSP_DEBUG

// Identifier for the chat network messages
const int MSG_CHAT = 153;
const int MSG_NODE_ERROR = 156;

#define INGAME_FONT "Fonts/m6x11.ttf"
#define INGAME_FONT2 "Fonts/SinsGold.ttf"

#define GAME_SERVER_ADDRESS "159.203.38.221"
//#define GAME_SERVER_ADDRESS "192.168.122.1"
//#define GAME_SERVER_ADDRESS "localhost"
//#define GAME_SERVER_ADDRESS "www.monkeymaya.com"

int numOfBoidsets = 10; // needs to be an even number for the boid splitting to work properly
int updateCycleIndex = 0;
BoidSet boids[10]; // 10 x 20 boids


URHO3D_DEFINE_APPLICATION_MAIN(MayaScape)

MayaScape::MayaScape(Context *context) :
        Game(context),
        loginClientObjectID_(0),
        isServer_(false),
        drawDebug_(false), csp_client(context) {

    Character2D::RegisterObject(context);
    Object2D::RegisterObject(context);
    Mover::RegisterObject(context);
    Vehicle::RegisterObject(context);
    RaycastVehicle::RegisterObject(context);
    WheelTrackModel::RegisterObject(context);
    Missile::RegisterObject(context);
    Bullet::RegisterObject(context);
    AP::RegisterObject(context);
    Player::RegisterObject(context);
    // register client objs
    ClientObj::RegisterObject(context);
    NetworkActor::RegisterObject(context);

    CSP_Server::RegisterObject(context);
    CSP_Client::RegisterObject(context);

}

void MayaScape::Setup() {
    Game::Setup();
}

void MayaScape::InitEvolutionSpriteGenerator() {

    std::cout << "Evolution Manager -> starting..." << std::endl;

    EvolutionManager::getInstance()->startEvolution();
    EvolutionManager *evolutionManager = EvolutionManager::getInstance();

    // Create shape for each agent
    std::vector<Agent *> agents = evolutionManager->getAgents();
    std::cout << "Evolution Manager -> number of agents = " << agents.size() << std::endl;

    for (int i = 0; i < agents.size(); i++) {
        // Randomly place agents
        //       agents[i]->setPosition(cyclone::Vector3(rd(), rd(), rd()));
    }

    // DISABLED CYCLONE PHYSICS CODE //
    /*
           // Create agent -> collision sphere
           cyclone::CollisionSphere cs;
           cs.body = new cyclone::RigidBody();
           agentCollSpheres.emplace_back(cs);

           cs.radius = 0.25f;
           cs.body->setMass(5.0f);
           cs.body->setDamping(0.9f, 0.9f);
           cyclone::Matrix3 it;
           it.setDiagonal(5.0f, 5.0f, 5.0f);
           cs.body->setPosition(agents[i]->getPosition());
           cs.body->setInertiaTensor(it);
           cs.body->setAcceleration(cyclone::Vector3::GRAVITY);

           cs.body->setCanSleep(false);
           cs.body->setAwake(true);
    */

    /*
        // Create the ball.
        agentCollSphere.body = new cyclone::RigidBody();
        ball.radius = 0.25f;
        ball.body->setMass(5.0f);
        ball.body->setDamping(0.9f, 0.9f);
        cyclone::Matrix3 it;
        it.setDiagonal(5.0f, 5.0f, 5.0f);
        ball.body->setInertiaTensor(it);
        ball.body->setAcceleration(cyclone::Vector3::GRAVITY);

        ball.body->setCanSleep(false);
        ball.body->setAwake(true);
    */
    /*
    // Test Case 1
    auto genotype = new Genotype();
    std::cout << "Generating random genotype..." << std::endl;
    genotype = genotype->generateRandom(10, 0.4, 20.4);
    genotype->outputToConsole();
    genotype->saveToFile("genotype01.data");
    std::cout << "Genotype saved to disk." << std::endl;

    std::cout << "Loading genotype from disk..." << std::endl;
    genotype = genotype->loadFromFile("genotype01.data");
    std::cout << "Loaded genotype from disk." << std::endl;
    */

}


void MayaScape::ShowEvolutionManagerStats() {
    std::vector<Agent *> agents = EvolutionManager::getInstance()->getAgents();
    std::vector<AgentController *> controllers = EvolutionManager::getInstance()->getAgentControllers();

    char buffer[255];
    int aliveCount = EvolutionManager::getInstance()->agentsAliveCount;
    const int maxRows = 20;
    char *strText[maxRows];

    for (int i = 0; i < maxRows; i++) {
        strText[i] = new char[80];
    }

    for (int i = 0; i < maxRows; i++) {

        switch (i) {

            case maxRows - 1:
                sprintf(strText[i], "%d alive out of %d population.", aliveCount, agents.size());
                break;
            case maxRows - 2:
                sprintf(strText[i], "%d generation out of %d generations.",
                        EvolutionManager::getGeneticAlgorithm()->generationCount, RestartAfter);
                break;
            case maxRows - 3:
                sprintf(strText[i], "==========================================================================");
                break;
            case maxRows - 4:
                sprintf(strText[i], "agent[0].timeSinceLastCheckpoint: %f",
                        controllers[0]->getTimeSinceLastCheckpoint());
                break;

            case maxRows - 5:
                sprintf(strText[i], "agent[0].x: %f, agent[0].y: %f, agent[0].z: %f", agents[0]->getPosition().x_,
                        agents[0]->getPosition().y_, agents[0]->getPosition().z_);
                break;
            case maxRows - 6:
                sprintf(strText[i], "agent[0].winX: %f, agent[0].winY: %f, agent[0].winZ: %f",
                        agents[0]->getWinPos().x_, agents[0]->getWinPos().y_, agents[0]->getWinPos().z_);
                break;
            case maxRows - 7:
                sprintf(strText[i], "", agents[0]->getPosition().z_);
                break;

            case maxRows - 8:
                sprintf(strText[i], "agent[0].evaluation: %f", agents[0]->genotype->evaluation);
                break;
            case maxRows - 9:
                sprintf(strText[i], "agent[0].horizontalInput: %f", controllers[0]->movement->getHorizontalInput());
                break;
            case maxRows - 10:
                sprintf(strText[i], "agent[0].verticalInput: %f", controllers[0]->movement->getVerticalInput());
                break;


            default:
                sprintf(strText[i], "");
                break;

        }

        //       if (strText[i])
//            renderText(5, 5 + (10 * i), strText[i], NULL);

    }

    for (int i = 0; i < maxRows; i++) {
        delete strText[i];
    }

    char *c = new char[255];
    for (int i = 0; i < agents.size(); i++) {

        sprintf(c, "Agent:  %s\nagent[%d].x: %f\nagent[%d].y: %f\nagent[%d].z: %f",
                agents[i]->getName(),
                i, agents[i]->getPosition().x_,
                i, agents[i]->getPosition().y_,
                i, agents[i]->getPosition().z_);

        // renderPanel(agents[i]->getWinPos().x_, agents[i]->getWinPos().y_, 200.0f, 100.0f, c);

        if (agents[i]->genotype) {
            //     renderParameters(agents[i]->getWinPos().x_ + 90.0f, agents[i]->getWinPos().y_ - 80.0f,
            //                      agents[i]->genotype->getParameterCopy());
        }
    }
    delete[] c;
}

void MayaScape::Start() {

    for (int i = 0; i < sizeof(particlePool_) / sizeof(*particlePool_); i++) {
        particlePool_[i].used = false;
        particlePool_[i].usedBy = -1;
    }

    // Don't start until past menu
    started_ = false;

    // Execute base class startup
    Game::Start();

    // rand seed
    SetRandomSeed(Time::GetSystemTime());

    // Initialize evolution sprite generator
    MayaScape::InitEvolutionSpriteGenerator();

    CreateServerSubsystem();

    context_->RegisterSubsystem(new GameController(context_));

    // Reset focus index
    focusIndex_ = 0;

    // Create the scene content
    CreateScene();

    // Generate UI client for network management
    CreateUI();

    SetupViewport();


    // Start in menu mode
    UpdateUIState(false);

    ResourceCache *cache = GetSubsystem<ResourceCache>();

    /*
    // Create boids
    for (int i = 0; i < numOfBoidsets; i++)
    {
        boids[i].Initialise(cache, scene_, Vector3(0.0f, 20.0f, 0.0f));
    }*/

    // targetCameraPos_ = Vector3(0.0f, 40.0f, CAMERA_DISTANCE);
    fpsTimer_.Reset();
    framesCount_ = 0;


    // Hook up to the frame update events
    SubscribeToEvents();

    ChangeDebugHudText();

    Game::InitMouseMode(MM_FREE);
}


void MayaScape::Stop() {

    // Free evolution manager
    EvolutionManager::clean();

    // Dump engine resources
    Game::Stop();
}

void MayaScape::CreateAgents() {

    ResourceCache *cache = GetSubsystem<ResourceCache>();

    float agentDropBoxSize = 200.0f;
    for (int i = 0; i < EvolutionManager::getInstance()->getAgents().size(); i++) {

        // Create AI player character
        //modelNode = sample2D_->CreateCharacter(0.0f, Vector3(3.5f + Random(-agentDropBoxSize, agentDropBoxSize), 80.0f, 0.0f + Random(-agentDropBoxSize, agentDropBoxSize)), 2);
        //agents_[i] = modelNode->CreateComponent<Character2D>(); // Create a logic component to handle character behavior
//        agents_[i]->agentIndex = i;
        Node *agentNode = scene_->CreateChild("Player");

        // Create the vehicle logic component
        agents_[i] = agentNode->CreateComponent<NetworkActor>();
//        agents_[i]->Init();
        agents_[i]->SetWaypoints((&waypointsWorld_));

        // Place on track
//        agents_[i]->GetNode()->SetPosition(Vector3(-814.0f + Random(-400.f, 400.0f), 400.0f, -595.0f + Random(-400.f, 400.0f)));

        // Store initial player position as focus
//        focusObjects_.Push(player_->GetNode()->GetPosition());

        // Place on at corner of map
//        TerrainPatch* p = terrain_->GetPatch(0, 0);
        //       IntVector2 v = p->GetCoordinates();

        //agents_[i]->GetNode()->SetRotation(Quaternion(0.0, -90.0, 0.0));


        // Create the vehicle logic component
        //   agents_[i]->vehicle_ = modelNode->CreateComponent<Vehicle>();
        // Create the rendering and physics components
        //    agents_[i]->vehicle_->Init();

//        String name = String("AI-Bear-P") + String(i);
        //      agents_[i]->GetNode()->SetName(name.CString());
        agents_[i]->isAI_ = true;
//        agents_[i]->playerPos_ = player_->GetNode()->GetPosition();
        agents_[i]->id_ = 1 + i;
        agents_[i]->type_ = 2;

//        agents_[i]->genotypeNode_ = scene_->CreateChild("Genotype " + i);
//       agents_[i]->powerbarNode_ = scene_->CreateChild("Powerbar " + i);

        // Get AI position
        Vector3 aiPos = agents_[i]->GetNode()->GetPosition();

        // Powerbar
        // Create billboard sets (powerbars)
        //const unsigned NUM_BILLBOARDNODES = 10;//NUM_AI;


        // A single billboard for each parameter of genotype
        const unsigned NUM_BILLBOARDS = EvolutionManager::getInstance()->getAgents()[0]->genotype->getParameterCount();

        // BillboardSet* billboardObject;

        Node *pbNode = scene_->CreateChild("PowerBar");
//        smokeNode->SetPosition(Vector3(Random(200.0f) - 100.0f, Random(20.0f) + 10.0f, Random(200.0f) - 100.0f));
        //pbNode->SetPosition(Vector3(3.5f+Random(-2.0f,2.0f), 20.0f, 0.0f));
        pbNode->SetPosition(Vector3(-0.02f, 0.25f, 0.0f));
//        pbNode->SetScale(Vector3(0.5f,0.5f,0.5f));
        auto *billboardObject = pbNode->CreateComponent<BillboardSet>(LOCAL);
        billboardObject->SetNumBillboards(NUM_BILLBOARDS);
        billboardObject->SetMaterial(cache->GetResource<Material>("Materials/PowerBar.xml"));
        billboardObject->SetSorted(true);

        for (unsigned j = 0; j < NUM_BILLBOARDS; ++j) {
            Billboard *bb = billboardObject->GetBillboard(j);
//            bb->position_ = Vector3(Random(12.0f) - 6.0f, Random(8.0f) - 4.0f, -5.0f);
            bb->position_ = Vector3(aiPos.x_, aiPos.y_, 0.0f);
            bb->size_ = Vector2((256.0f / 512.0f) * 0.06f, (256.0f / 144.0f) * 0.06f);
            bb->rotation_ = 90.0f; //Random() * 360.0f;
            bb->enabled_ = true;

//            bb->uv_ = Rect(left,top,right,bottom);
            bb->uv_ = Rect(0.0, 0.5, 1.0, 1.0);

            // After modifying the billboards, they need to be "committed" so that the BillboardSet updates its internals
            billboardObject->Commit();
        }

/*
        // Genotype

        //Node* gtNode = scene_->CreateChild("Genotype");
        //pbNode->SetPosition(Vector3(3.5f+Random(-2.0f,2.0f), 20.0f, 0.0f));

        agents_[i]->genotypeNode_->SetPosition(Vector3(-0.24f, 0.25f, 0.0f));
//        pbNode->SetScale(Vector3(0.5f,0.5f,0.5f));
        agents_[i]->genotypeBBSet_ = agents_[i]->genotypeNode_->CreateComponent<BillboardSet>();
        agents_[i]->genotypeBBSet_->SetNumBillboards(NUM_BILLBOARDS);
        agents_[i]->genotypeBBSet_->SetMaterial(cache->GetResource<Material>("Materials/Genotype.xml"));
        agents_[i]->genotypeBBSet_->SetSorted(true);

        // Draw billboard for each genotype parameter -> alpha based on value

        for (unsigned j = 0; j < NUM_BILLBOARDS; ++j) {
            Billboard *bb = agents_[i]->genotypeBBSet_->GetBillboard(j);

            // Get genotype parameters
            for (int g = 0; g < EvolutionManager::getInstance()->getAgents()[i]->genotype->getParameterCount(); g++) {
                float parameter = EvolutionManager::getInstance()->getAgents()[i]->genotype->getParameter(g);

                // Set alpha based on parameter
               //bb->color_ = Urho3D::Color(parameter, parameter, parameter);

                //URHO3D_LOGINFOF("Agent [%d]: genotype parameter[%d] -> %f", i, g, parameter);

                //float fx = 0.5;
                //	float fy = fx / (m_i_width / m_i_height);
                //	m_bb->size_ = Vector2(fx, fy);

                // Diminish height of genotype billboard by parameter value
                bb->size_ = Vector2((1.0f) * 0.05f * parameter, (0.1f) * 0.05f);
            }

//            bb->size_ = Vector2((256.0f/8.0f)*0.06f, (256.0f/144.0f)*0.06f);
        //    bb->size_ = Vector2((1.0f) * 0.05f, (0.1f) * 0.05f);

            //float fx = 0.5;
            //	float fy = fx / (m_i_width / m_i_height);
            //	m_bb->size_ = Vector2(fx, fy);

            bb->rotation_ = 90.0f; //Random() * 360.0f;
            bb->enabled_ = true;

//            bb->uv_ = Rect(left,top,right,bottom);
            bb->uv_ = Rect(0.0, 0.0, 1.0, 1.0);

            // After modifying the billboards, they need to be "committed" so that the BillboardSet updates its internals
            agents_[i]->genotypeBBSet_->Commit();
        }
*/
        // Powerbar

        /*
        agents_[i]->powerbarNode_->SetPosition(Vector3(0.0, 0.25f, 0.0f));
        agents_[i]->powerbarBBSet_ = agents_[i]->powerbarNode_->CreateComponent<BillboardSet>();
        agents_[i]->powerbarBBSet_->SetNumBillboards(1);
        agents_[i]->powerbarBBSet_->SetMaterial(cache->GetResource<Material>("Materials/PowerBar.xml"));
        agents_[i]->powerbarBBSet_->SetSorted(true);

        // Draw billboard for each genotype parameter -> alpha based on value

        // Single billboard for power meter
        for (unsigned j = 0; j < 1; ++j) {
            Billboard *bb = agents_[i]->powerbarBBSet_->GetBillboard(j);

//            bb->size_ = Vector2((256.0f/8.0f)*0.06f, (256.0f/144.0f)*0.06f);
            bb->size_ = Vector2((1.0f) * 0.05f, (0.1f) * 0.05f);

            bb->rotation_ = 90.0f; //Random() * 360.0f;
            bb->enabled_ = true;

//            bb->uv_ = Rect(left,top,right,bottom);
            bb->uv_ = Rect(0.0, 0.0, 1.0, 1.0);

            // After modifying the billboards, they need to be "committed" so that the BillboardSet updates its internals
            agents_[i]->powerbarBBSet_->Commit();
        }
    }*/
    }
}


void MayaScape::CreateScene() {

    ResourceCache *cache = GetSubsystem<ResourceCache>();
    // Create scene subsystem components
    scene_ = new Scene(context_);

    // server requires client hash and scene info
    Server *server = GetSubsystem<Server>();

    // Register Network Actor and local client scene
    server->RegisterClientHashAndScene(NetworkActor::GetTypeStatic(), scene_);
    URHO3D_LOGINFOF("Local: Scene checksum -> %d", ToStringHex(scene_->GetChecksum()).CString());

    // Create octree and physics world with default settings
    // Create them as local so that they are not needlessly replicated when a client connects
    scene_->CreateComponent<Octree>(LOCAL);
    PhysicsWorld *physicsWorld = scene_->CreateComponent<PhysicsWorld>(LOCAL);
    DebugRenderer *dbgRenderer = scene_->CreateComponent<DebugRenderer>(LOCAL);
    physicsWorld->SetDebugRenderer(dbgRenderer);

    // Disable interpolation (need determinism)
    physicsWorld->SetInterpolation(false);
#ifdef CSP_DEBUG
    physicsWorld->SetFps(10);
#endif

    //static const Vector3 NEW_GRAVITY = Vector3(0.0f, 0.0f, 0.0f);
    //physicsWorld->SetGravity(NEW_GRAVITY);

    // Create camera and define viewport. We will be doing load / save, so it's convenient to create the camera outside the scene,
    // so that it won't be destroyed and recreated, and we don't have to redefine the viewport on load
    cameraNode_ = scene_->CreateChild("Camera", LOCAL);
//    cameraNode_ = new Node(context_);
    auto *camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFov(60);
    camera->SetFarClip(500.0f);
    cameraNode_->SetPosition(Vector3(0.0f, 5.0f, 0.0f));

    // On client
    if (!isServer_) {

        // Enable for 3D sounds to work (attach to camera node)
        SoundListener *listener = cameraNode_->CreateComponent<SoundListener>();
        GetSubsystem<Audio>()->SetListener(listener);

        // you can set master volumes for the different kinds if sounds, here 30% for music
        GetSubsystem<Audio>()->SetMasterGain(SOUND_MUSIC, 1.2);

        // Create a directional light with shadows
        Node *lightNode = scene_->CreateChild("DirectionalLight", LOCAL);
        lightNode->SetDirection(Vector3(0.3f, -0.5f, 0.425f));
        Light *light = lightNode->CreateComponent<Light>();
        light->SetLightType(LIGHT_DIRECTIONAL);
        light->SetCastShadows(true);
        light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
        light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
        light->SetSpecularIntensity(0.5f);

        // All static scene content and the camera are also created as local,
        // so that they are unaffected by scene replication and are
        // not removed from the client upon connection.
        // Create a Zone component first for ambient lighting & fog control.

        // Create static scene content. First create a zone for ambient lighting and fog control
        Node *zoneNode = scene_->CreateChild("Zone", LOCAL);
        Zone *zone = zoneNode->CreateComponent<Zone>();
        zone->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
        zone->SetFogColor(Color(0.5f, 0.5f, 0.7f));
        zone->SetFogStart(700.0f);
        zone->SetFogEnd(900.0f);
        zone->SetBoundingBox(BoundingBox(-2000.0f, 2000.0f));


        auto *graphics = GetSubsystem<Graphics>();

        //camera->SetOrthographic(true);
        //camera->SetOrthoSize((float)graphics->GetHeight() * PIXEL_SIZE);
        camera->SetZoom(0.1f * Min((float) graphics->GetWidth() / 1080.0f, (float) graphics->GetHeight() /
                                                                           768.0f)); // Set zoom according to user's resolution to ensure full visibility (initial zoom (2.0) is set for full visibility at 1280x800 resolution)
        camera->SetFarClip(1000.0f);

        UI *ui = GetSubsystem<UI>();

        packetsIn_ = ui->GetRoot()->CreateChild<Text>();
        packetsIn_->SetText("Packets in : 0");
        packetsIn_->SetFont(cache->GetResource<Font>(INGAME_FONT), 10);
        packetsIn_->SetHorizontalAlignment(HA_RIGHT);
        packetsIn_->SetVerticalAlignment(VA_CENTER);
        packetsIn_->SetPosition(-50, -10);

        packetsOut_ = ui->GetRoot()->CreateChild<Text>();
        packetsOut_->SetText("Packets out: 0");
        packetsOut_->SetFont(cache->GetResource<Font>(INGAME_FONT), 10);
        packetsOut_->SetHorizontalAlignment(HA_RIGHT);
        packetsOut_->SetVerticalAlignment(VA_CENTER);
        packetsOut_->SetPosition(-50, 10);


        // Set the default UI style and font
        //ui->GetRoot()->SetDefaultStyle(cache->GetResource<XMLFile>("UI/DefaultStyle.xml"));
        auto *font = cache->GetResource<Font>(INGAME_FONT2);

        // Get powerbar texture
        Texture2D *powerBarTexture = cache->GetResource<Texture2D>("Textures/powerbar.png");
        if (!powerBarTexture)
            return;

        // Get powerbar background texture
        Texture2D *powerBarBkgTexture = cache->GetResource<Texture2D>("Textures/powerbar-bk.png");
        if (!powerBarBkgTexture)
            return;

        // Get RPM bar texture
        Texture2D *rpmBarTexture = cache->GetResource<Texture2D>("Textures/rpm.png");
        if (!rpmBarTexture)
            return;

        // Get RPM bar background texture
        Texture2D *rpmBarBkgTexture = cache->GetResource<Texture2D>("Textures/powerbar-bk.png");
        if (!rpmBarBkgTexture)
            return;

        // Get velocity bar texture
        Texture2D *velBarTexture = cache->GetResource<Texture2D>("Textures/velocity.png");
        if (!rpmBarTexture)
            return;

        // Get velocity bar background texture
        Texture2D *velBarBkgTexture = cache->GetResource<Texture2D>("Textures/powerbar-bk.png");
        if (!rpmBarBkgTexture)
            return;

        // Get mini map p1 texture
        Texture2D *miniMapP1Texture = cache->GetResource<Texture2D>("Textures/minimap-p1.png");
        if (!miniMapP1Texture)
            return;

        // Get mini map waypoint texture
        Texture2D *miniMapWPTexture = cache->GetResource<Texture2D>("Textures/minimap-wp.png");
        if (!miniMapWPTexture)
            return;

        // Get mini map background texture
        Texture2D *miniMapBkgTexture = cache->GetResource<Texture2D>("Textures/minimap-bk.png");
        if (!miniMapBkgTexture)
            return;

        // Get marker map background texture
        Texture2D *markerMapBkgTexture = cache->GetResource<Texture2D>("Textures/MarkerMap.png");
        if (!markerMapBkgTexture)
            return;

        // Get genotype texture
        Texture2D *genotypeTexture = cache->GetResource<Texture2D>("Textures/genotype.png");
        if (!genotypeTexture)
            return;

        // Get genotype background texture
        Texture2D *genotypeBkgTexture = cache->GetResource<Texture2D>("Textures/genotype-bk.png");
        if (!genotypeBkgTexture)
            return;

        // Get steering wheel texture
        Texture2D *steerWheelTexture = cache->GetResource<Texture2D>("Textures/steer-wheel.png");
        if (!steerWheelTexture)
            return;

        // Create sprite and add to the UI layout
        powerBarP1Sprite_ = ui->GetRoot()->CreateChild<Sprite>();
        powerBarBkgP1Sprite_ = ui->GetRoot()->CreateChild<Sprite>();
        rpmBarP1Sprite_ = ui->GetRoot()->CreateChild<Sprite>();
        rpmBarBkgP1Sprite_ = ui->GetRoot()->CreateChild<Sprite>();
        velBarP1Sprite_ = ui->GetRoot()->CreateChild<Sprite>();
        velBarBkgP1Sprite_ = ui->GetRoot()->CreateChild<Sprite>();
        miniMapP1Sprite_ = ui->GetRoot()->CreateChild<Sprite>();
        miniMapWPSprite_ = ui->GetRoot()->CreateChild<Sprite>();
        miniMapBkgSprite_ = ui->GetRoot()->CreateChild<Sprite>();
        markerMapBkgSprite_ = ui->GetRoot()->CreateChild<Sprite>();
        steerWheelSprite_ = ui->GetRoot()->CreateChild<Sprite>();

        // Set sprite texture
        powerBarP1Sprite_->SetTexture(powerBarTexture);
        powerBarBkgP1Sprite_->SetTexture(powerBarBkgTexture);
        rpmBarP1Sprite_->SetTexture(rpmBarTexture);
        rpmBarBkgP1Sprite_->SetTexture(rpmBarBkgTexture);
        velBarP1Sprite_->SetTexture(velBarTexture);
        velBarBkgP1Sprite_->SetTexture(rpmBarBkgTexture);
        miniMapP1Sprite_->SetTexture(miniMapP1Texture);
        miniMapWPSprite_->SetTexture(miniMapWPTexture);
        miniMapBkgSprite_->SetTexture(miniMapBkgTexture);
        markerMapBkgSprite_->SetTexture(markerMapBkgTexture);
        steerWheelSprite_->SetTexture(steerWheelTexture);

        float textOverlap = 245.0f;

        powerBarText_ = ui->GetRoot()->CreateChild<Text>("powerBarText");
        powerBarText_->SetAlignment(HA_LEFT, VA_TOP);
        powerBarText_->SetPosition(300.0f - textOverlap, 20.0);
        powerBarText_->SetFont(font, 15);
        powerBarText_->SetTextEffect(TE_SHADOW);
        powerBarText_->SetText(String("HEALTH"));
        powerBarText_->SetVisible(false);

        int textureWidth;
        int textureHeight;

        textureWidth = powerBarTexture->GetWidth();
        textureHeight = powerBarTexture->GetHeight();

        powerBarP1Sprite_->SetScale(256.0f / textureWidth);
        powerBarP1Sprite_->SetSize(textureWidth, textureHeight);
        powerBarP1Sprite_->SetHotSpot(textureWidth, textureHeight);
        powerBarP1Sprite_->SetAlignment(HA_LEFT, VA_TOP);
        powerBarP1Sprite_->SetPosition(Vector2(300.0f, 50.0f));
        powerBarP1Sprite_->SetOpacity(1.0f);
        // Set a low priority so that other UI elements can be drawn on top
        powerBarP1Sprite_->SetPriority(-100);

        powerBarBkgP1Sprite_->SetScale(256.0f / textureWidth);
        powerBarBkgP1Sprite_->SetSize(textureWidth, textureHeight);
        powerBarBkgP1Sprite_->SetHotSpot(textureWidth, textureHeight);
        powerBarBkgP1Sprite_->SetAlignment(HA_LEFT, VA_TOP);
        powerBarBkgP1Sprite_->SetPosition(Vector2(300.0f, 50.0f));
        powerBarBkgP1Sprite_->SetOpacity(0.2f);
        // Set a low priority so that other UI elements can be drawn on top
        powerBarBkgP1Sprite_->SetPriority(-100);

        powerBarP1Sprite_->SetVisible(true);
        powerBarBkgP1Sprite_->SetVisible(true);

        rpmBarText_ = ui->GetRoot()->CreateChild<Text>("rpmBarText");
        rpmBarText_->SetAlignment(HA_LEFT, VA_TOP);
        rpmBarText_->SetPosition(300.0f - textOverlap, 170.0);
        rpmBarText_->SetFont(font, 15);
        rpmBarText_->SetTextEffect(TE_SHADOW);
        rpmBarText_->SetText(String("RPM"));
        rpmBarText_->SetVisible(false);


        textureWidth = rpmBarTexture->GetWidth();
        textureHeight = rpmBarTexture->GetHeight();

        rpmBarP1Sprite_->SetScale(256.0f / textureWidth);
        rpmBarP1Sprite_->SetSize(textureWidth, textureHeight);
        rpmBarP1Sprite_->SetHotSpot(textureWidth, textureHeight);
        rpmBarP1Sprite_->SetAlignment(HA_LEFT, VA_TOP);
        rpmBarP1Sprite_->SetPosition(Vector2(300.0f, 200.0f));
        rpmBarP1Sprite_->SetOpacity(1.0f);
        // Set a low priority so that other UI elements can be drawn on top
        rpmBarP1Sprite_->SetPriority(-100);

        rpmBarBkgP1Sprite_->SetScale(256.0f / textureWidth);
        rpmBarBkgP1Sprite_->SetSize(textureWidth, textureHeight);
        rpmBarBkgP1Sprite_->SetHotSpot(textureWidth, textureHeight);
        rpmBarBkgP1Sprite_->SetAlignment(HA_LEFT, VA_TOP);
        rpmBarBkgP1Sprite_->SetPosition(Vector2(300.0f, 200.0f));
        rpmBarBkgP1Sprite_->SetOpacity(0.2f);
        // Set a low priority so that other UI elements can be drawn on top
        rpmBarBkgP1Sprite_->SetPriority(-100);

        rpmBarP1Sprite_->SetVisible(true);
        rpmBarBkgP1Sprite_->SetVisible(true);

        velBarText_ = ui->GetRoot()->CreateChild<Text>("velBarText");
        velBarText_->SetAlignment(HA_LEFT, VA_TOP);
        velBarText_->SetPosition(300.0f - textOverlap, 90.0);
        velBarText_->SetFont(font, 15);
        velBarText_->SetTextEffect(TE_SHADOW);
        velBarText_->SetText(String("SPEED"));
        velBarText_->SetVisible(false);

        textureWidth = rpmBarTexture->GetWidth();
        textureHeight = rpmBarTexture->GetHeight();

        velBarP1Sprite_->SetScale(256.0f / textureWidth);
        velBarP1Sprite_->SetSize(textureWidth, textureHeight);
        velBarP1Sprite_->SetHotSpot(textureWidth, textureHeight);
        velBarP1Sprite_->SetAlignment(HA_LEFT, VA_TOP);
        velBarP1Sprite_->SetPosition(Vector2(300.0f, 120.0f));
        velBarP1Sprite_->SetOpacity(1.0f);
        // Set a low priority so that other UI elements can be drawn on top
        velBarP1Sprite_->SetPriority(-100);

        velBarBkgP1Sprite_->SetScale(256.0f / textureWidth);
        velBarBkgP1Sprite_->SetSize(textureWidth, textureHeight);
        velBarBkgP1Sprite_->SetHotSpot(textureWidth, textureHeight);
        velBarBkgP1Sprite_->SetAlignment(HA_LEFT, VA_TOP);
        velBarBkgP1Sprite_->SetPosition(Vector2(300.0f, 120.0f));
        velBarBkgP1Sprite_->SetOpacity(0.2f);
        // Set a low priority so that other UI elements can be drawn on top
        velBarBkgP1Sprite_->SetPriority(-100);

        velBarP1Sprite_->SetVisible(true);
        velBarBkgP1Sprite_->SetVisible(true);


        textureWidth = miniMapP1Texture->GetWidth();
        textureHeight = miniMapP1Texture->GetHeight();

        //   float miniMapP1X = 776.0f+45.0f;
        //   float miniMapP1Y = 300.0f-15.0f;

        miniMapP1Sprite_->SetScale(0.4);//256.0f / textureWidth);
        miniMapP1Sprite_->SetSize(textureWidth, textureHeight);
        miniMapP1Sprite_->SetHotSpot(textureWidth / 2, textureHeight / 2);
        miniMapP1Sprite_->SetAlignment(HA_LEFT, VA_TOP);
//    miniMapP1Sprite_->SetPosition(Vector2(miniMapP1X-16.0f, miniMapP1Y));
//    miniMapP1Sprite_->SetPosition(Vector2(miniMapP1X+256.0f-16.0f, miniMapP1Y));

        miniMapP1Sprite_->SetOpacity(0.9f);
        // Set a low priority so that other UI elements can be drawn on top
        miniMapP1Sprite_->SetPriority(-100);


        textureWidth = miniMapWPTexture->GetWidth();
        textureHeight = miniMapWPTexture->GetHeight();

        //   float miniMapP1X = 776.0f+45.0f;
        //   float miniMapP1Y = 300.0f-15.0f;

        miniMapWPSprite_->SetScale(0.4);//256.0f / textureWidth);
        miniMapWPSprite_->SetSize(textureWidth, textureHeight);
        miniMapWPSprite_->SetHotSpot(textureWidth / 2, textureHeight / 2);
        miniMapWPSprite_->SetAlignment(HA_LEFT, VA_TOP);
//    miniMapP1Sprite_->SetPosition(Vector2(miniMapP1X-16.0f, miniMapP1Y));
//    miniMapP1Sprite_->SetPosition(Vector2(miniMapP1X+256.0f-16.0f, miniMapP1Y));

        miniMapWPSprite_->SetOpacity(0.9f);
        // Set a low priority so that other UI elements can be drawn on top
        miniMapWPSprite_->SetPriority(-100);


        textureWidth = miniMapBkgTexture->GetWidth();
        textureHeight = miniMapBkgTexture->GetHeight();

        float miniMapX = 1000.0f + 45.0f;
        float miniMapY = 300.0f - 15.0f;

        miniMapBkgSprite_->SetScale(256.0f / textureWidth);
        miniMapBkgSprite_->SetSize(textureWidth, textureHeight);
        miniMapBkgSprite_->SetHotSpot(textureWidth, textureHeight);
        miniMapBkgSprite_->SetAlignment(HA_LEFT, VA_TOP);
        miniMapBkgSprite_->SetPosition(Vector2(miniMapX, miniMapY));
        miniMapBkgSprite_->SetOpacity(0.2f);
        // Set a low priority so that other UI elements can be drawn on top
        miniMapBkgSprite_->SetPriority(-100);

        miniMapP1Sprite_->SetVisible(true);
        miniMapWPSprite_->SetVisible(true);
        miniMapBkgSprite_->SetVisible(true);


        textureWidth = markerMapBkgTexture->GetWidth();
        textureHeight = markerMapBkgTexture->GetHeight();

        markerMapBkgSprite_->SetScale(256.0f / textureWidth);
        markerMapBkgSprite_->SetSize(textureWidth, textureHeight);
        markerMapBkgSprite_->SetHotSpot(textureWidth, textureHeight);
        markerMapBkgSprite_->SetAlignment(HA_LEFT, VA_TOP);
        markerMapBkgSprite_->SetPosition(Vector2(miniMapX, miniMapY));
        markerMapBkgSprite_->SetOpacity(0.5f);
        // Set a low priority so that other UI elements can be drawn on top
        miniMapBkgSprite_->SetPriority(-100);
        markerMapBkgSprite_->SetVisible(true);


        textureWidth = steerWheelTexture->GetWidth();
        textureHeight = steerWheelTexture->GetHeight();

        float steerWheelX = 900.0f;
        float steerWheelY = 600.0f - 15.0f;

        steerWheelSprite_->SetScale(256.0f / textureWidth);
        steerWheelSprite_->SetSize(textureWidth, textureHeight);
        steerWheelSprite_->SetHotSpot(textureWidth / 2, textureHeight / 2);
        steerWheelSprite_->SetAlignment(HA_LEFT, VA_TOP);
        steerWheelSprite_->SetPosition(Vector2(steerWheelX, steerWheelY));
        steerWheelSprite_->SetOpacity(0.5f);
        // Set a low priority so that other UI elements can be drawn on top
        steerWheelSprite_->SetPriority(-100);
        steerWheelSprite_->SetVisible(true);

        // Debug text
        for (int i = 0; i < NUM_DEBUG_FIELDS; i++) {
            debugText_[i] = ui->GetRoot()->CreateChild<Text>("DebugText");
            debugText_[i]->SetAlignment(HA_LEFT, VA_CENTER);
            debugText_[i]->SetPosition(10.0f, 500.0 + (i * 20));
            debugText_[i]->SetFont(font, 24);
            debugText_[i]->SetTextEffect(TE_SHADOW);
            debugText_[i]->SetVisible(true);
            std::string debugData1;
            debugData1.append("-");
            debugText_[i]->SetText(debugData1.c_str());
        }
    }

    using namespace std;

// Create heightmap terrain with collision
    Node *terrainNode = scene_->CreateChild("Terrain", LOCAL);
    terrainNode->SetPosition(Vector3::ZERO);
    terrain_ = terrainNode->CreateComponent<Terrain>();
    terrain_->SetPatchSize(64);
    terrain_->SetSpacing(Vector3(2.8f, 0.2f, 2.8f));
//    terrain->SetSpacing(Vector3(3.0f, 0.1f, 3.0f)); // Spacing between vertices and vertical resolution of the height map

    //    terrain->SetHeightMap(cache->GetResource<Image>("Offroad/Terrain/HeightMapRace-257.png"));
//    terrain->SetMaterial(cache->GetResource<Material>("Offroad/Terrain/TerrainRace-256.xml"));
    terrain_->SetMarkerMap(cache->GetResource<Image>("Textures/MarkerMap.png"));
    terrain_->SetHeightMap(cache->GetResource<Image>("Textures/HeightMap.png"));
    terrain_->SetMaterial(cache->GetResource<Material>("Materials/Terrain.xml"));

    terrain_->SetOccluder(true);

    // TRACK MARKER
    // HSL -> 0.500000059604645,1,0.643137276172638

    // Search for track marker
    int trackX = 0;
    int trackY = 0;
    for (int k = 0; k < terrain_->GetMarkerMap()->GetHeight(); k++) {
        for (int j = 0; j < terrain_->GetMarkerMap()->GetWidth(); j++) {
            Vector3 hsl_ = terrain_->GetMarkerMap()->GetPixel(k, j).ToHSL();
            //URHO3D_LOGINFOF("terrain marker map[x,y]=[%f,%f,%f]", j, k, hsl_.x_, hsl_.y_, hsl_.z_);


            if (hsl_ == bkgMarkerToken)
                continue;

            if (hsl_ == treeMarkerToken) {
                trees_.Push(Vector3((float) j, 0.0f, (float) k));
            } else if (hsl_ == trackMarkerToken) {
                trackX = j;
                trackY = k;
            } else if (hsl_ == waypointToken) {
                waypoints_.Push((Vector3((float) j, 0.0f, (float) k)));
            } else {
                // Store track marker
                URHO3D_LOGINFOF("***** UNKNOWN terrain marker map[%d,%d]=[%f,%f,%f]", j, k, hsl_.x_, hsl_.y_, hsl_.z_);
            }

        }
    }

    int reduceFactor = ((float) trees_.Size() * 1.99f);
    // Drop trees to reduce saturation of trees
    int reduceSize = Min(trees_.Size(), reduceFactor);

    for (int j = 0; j < reduceSize; j++) {
        trees_.Erase(Random(0, trees_.Size()), 1);
    }

    URHO3D_LOGINFOF("***** TREE COUNT: [%d]", trees_.Size());
    URHO3D_LOGINFOF("***** WAYPOINT COUNT: [%d]", waypoints_.Size());

    RigidBody *body = terrainNode->CreateComponent<RigidBody>(LOCAL);
    body->SetCollisionLayer(NETWORKACTOR_COL_LAYER); // Use layer bitmask 2 for static geometry
    CollisionShape *shape = terrainNode->CreateComponent<CollisionShape>(LOCAL);

    // Assigns terrain collision map (calculated based on heightmap)
    shape->SetTerrain();

    // Load race track at track marker

    // Convert marker position to world position for track
    int w = terrain_->GetMarkerMap()->GetWidth();
    int h = terrain_->GetMarkerMap()->GetHeight();


    float trackOffsetX = -mapSize / 2;
    float trackOffsetY = -mapSize / 2;
    float trackPosX = (((float) trackX / (float) w) * mapSize) + trackOffsetX;
    float trackPosZ = (((float) trackY / (float) h) * mapSize) + trackOffsetY;

    // Convert from mini map to world position
//    Vector3 shiftedRange = Vector3(trackPosX, 0, trackPosZ) - Vector3(mapSize/2, mapSize/2, mapSize/2);

    URHO3D_LOGINFOF("-----> SET RACE TRACK TO location in world space [%f,%f,%f]", trackPosX, 0.0f, trackPosZ);

    /*
    // 1600+1600
    float xRange = (shiftedRange.x_*mapSize) / miniMapWidth;
    float zRange = (shiftedRange.z_*mapSize) / miniMapHeight;
*/

    raceTrack_ = scene_->CreateChild("RaceTrack", LOCAL);
    Vector3 position(trackPosX, 0.0f, trackPosZ);
    position.y_ = terrain_->GetHeight(position) + 20.0f;
    raceTrack_->SetPosition(position);
    // Create a rotation quaternion from up vector to terrain normal
    //raceTrack_->SetRotation(Quaternion(Vector3::UP, terrain_->GetNormal(position)));
    Node *adjNode = raceTrack_->CreateChild("AdjNode", LOCAL);
    adjNode->SetRotation(Quaternion(0.0, 0.0, 0.0f));

    raceTrack_->SetScale(30.0f);

    auto *object = adjNode->CreateComponent<StaticModel>(LOCAL);
    std::string mdlPath = "Models/Tracks/Models/trackA.mdl";
    std::string matPath = "Models/Tracks/Models/trackA.txt";
    auto *model = cache->GetResource<Model>(mdlPath.c_str());
    object->SetModel(model);
    object->ApplyMaterialList(matPath.c_str());
    object->SetCastShadows(true);
    object->SetEnabled(false);

    body = adjNode->CreateComponent<RigidBody>(LOCAL);
    body->SetCollisionLayer(2);
    trackColShape_ = adjNode->CreateComponent<CollisionShape>(LOCAL);
    trackColShape_->SetTriangleMesh(object->GetModel(), 0);
//        trackColShape_->SetConvexHull(model);


    // Place trees based on markers



    // Convert from mini map to world position
//    Vector3 shiftedRange = Vector3(trackPosX, 0, trackPosZ) - Vector3(mapSize/2, mapSize/2, mapSize/2);

    URHO3D_LOGINFOF("-----> SET RACE TRACK TO location in world space [%f,%f,%f]", trackPosX, 0.0f, trackPosZ);

    // Place trees
    for (unsigned i = 0; i < trees_.Size(); ++i) {
        float treeOffsetX = -mapSize / 2;
        float treeOffsetY = -mapSize / 2;
        // Convert marker position to world position for track
        float treePosX =
                (((float) trees_[i].x_ / (float) terrain_->GetMarkerMap()->GetWidth()) * mapSize) + treeOffsetX;
        float treePosZ =
                (((float) trees_[i].z_ / (float) terrain_->GetMarkerMap()->GetHeight()) * mapSize) + treeOffsetY;

        Node *objectNode = scene_->CreateChild("Tree", LOCAL);
        Vector3 position(treePosX, 0.0f, treePosZ);
        position.y_ = terrain_->GetHeight(position) - 0.1f;
        objectNode->SetPosition(position);
        objectNode->SetEnabled(false);

        // Store tree position as focus
        focusObjects_.Push(position);

        // Create a rotation quaternion from up vector to terrain normal
        objectNode->SetRotation(Quaternion(Vector3::UP, terrain_->GetNormal(position)));
        Node *adjNode = objectNode->CreateChild("AdjNode", LOCAL);
        adjNode->SetRotation(Quaternion(0.0, 0.0, -90.0f));

        objectNode->SetScale(20.0f);

        auto *object = adjNode->CreateComponent<StaticModel>(LOCAL);

        // Random
        int r = std::round(Random(0.0f, 5.0f));
        switch (r) {
            case 0:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-baobab_orange.mdl"));
                break;
            case 1:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-birch02.mdl"));
                break;
            case 2:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-elipse.mdl"));
                break;
            case 3:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-fir.mdl"));
                break;
            case 4:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-oak.mdl"));
                break;
            case 5:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-lime.mdl"));
                break;
        }

        //       object->SetMaterial(cache->GetResource<Material>("Materials/LOWPOLY_COLORS.xml")
        object->SetMaterial(cache->GetResource<Material>("Materials/LOWPOLY-COLORS.xml"));
        object->SetCastShadows(true);

        auto *body = adjNode->CreateComponent<RigidBody>(LOCAL);
        body->SetCollisionLayer(2);
        auto *shape = objectNode->CreateComponent<CollisionShape>(LOCAL);
        shape->SetTriangleMesh(object->GetModel(), 0);
    }

    // Place waypoints

    // Store focusObjects index for waypoint reference
    wpStartIndex_ = focusObjects_.Size() + 1;
    for (unsigned i = 0; i < waypoints_.Size(); ++i) {
        float wpOffsetX = -mapSize / 2;
        float wpOffsetY = -mapSize / 2;
        // Convert marker position to world position for track
        float wpPosX =
                (((float) waypoints_[i].x_ / (float) terrain_->GetMarkerMap()->GetWidth()) * mapSize) + wpOffsetX;
        float wpPosZ =
                (((float) waypoints_[i].z_ / (float) terrain_->GetMarkerMap()->GetHeight()) * mapSize) + wpOffsetY;
//terrain_->GetHeight(Vector3(wpPosX, 0.0f, wpPosZ))
        waypointsWorld_.Push(Vector3(wpPosX, 0, wpPosZ));

        Node *objectNode = scene_->CreateChild("Waypoint", LOCAL);
        Vector3 position(wpPosX, 0.0f, wpPosZ);
        position.y_ = terrain_->GetHeight(position) - 0.1f;
        objectNode->SetPosition(position);

        // Store tree position as focus
        focusObjects_.Push(position);

        // Create a rotation quaternion from up vector to terrain normal
        objectNode->SetRotation(Quaternion(Vector3::UP, terrain_->GetNormal(position)));
        Node *adjNode = objectNode->CreateChild("AdjNode", LOCAL);
        adjNode->SetRotation(Quaternion(0.0, 0.0, -90.0f));

        objectNode->SetScale(20.0f);

        auto *object = adjNode->CreateComponent<StaticModel>(LOCAL);
        object->SetModel(cache->GetResource<Model>("Models/AssetPack/castle-flag.mdl"));

        //       object->SetMaterial(cache->GetResource<Material>("Materials/LOWPOLY_COLORS.xml")
        object->SetMaterial(cache->GetResource<Material>("Materials/LOWPOLY-COLORS.xml"));
        object->SetCastShadows(true);
        object->SetEnabled(false);

        auto *body = adjNode->CreateComponent<RigidBody>(LOCAL);
        body->SetCollisionLayer(2);
        auto *shape = objectNode->CreateComponent<CollisionShape>(LOCAL);
        shape->SetTriangleMesh(object->GetModel(), 0);
    }




    //
/*
    // Create 1000 mushrooms in the terrain. Always face outward along the terrain normal
    const unsigned NUM_MUSHROOMS = 1000;
    for (unsigned i = 0; i < NUM_MUSHROOMS; ++i)
    {
        Node* objectNode = scene_->CreateChild("Mushroom");
        Vector3 position(Random(2000.0f) - 1000.0f, 0.0f, Random(2000.0f) - 1000.0f);
        position.y_ = terrain_->GetHeight(position) - 0.1f;
        objectNode->SetPosition(position);
        // Create a rotation quaternion from up vector to terrain normal
        objectNode->SetRotation(Quaternion(Vector3::UP, terrain_->GetNormal(position)));
        Node* adjNode = objectNode->CreateChild("AdjNode");
        adjNode->SetRotation(Quaternion(0.0, 0.0, -90.0f));

        objectNode->SetScale(3.5f);

        auto* object = adjNode->CreateComponent<StaticModel>();

        // Random
        int r = std::round(Random(0.0f, 5.0f));
        switch (r) {
            case 0:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-baobab_orange.mdl"));
                break;
            case 1:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-birch02.mdl"));
                break;
            case 2:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-elipse.mdl"));
                break;
            case 3:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-fir.mdl"));
                break;
            case 4:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-oak.mdl"));
                break;
            case 5:
                object->SetModel(cache->GetResource<Model>("Models/AssetPack/tree-lime.mdl"));
                break;
        }

 //       object->SetMaterial(cache->GetResource<Material>("Materials/LOWPOLY_COLORS.xml")
        object->SetMaterial(cache->GetResource<Material>("Materials/LOWPOLY-COLORS.xml"));
        object->SetCastShadows(true);

        auto* body = adjNode->CreateComponent<RigidBody>();
        body->SetCollisionLayer(2);
        auto* shape = objectNode->CreateComponent<CollisionShape>();
        shape->SetTriangleMesh(object->GetModel(), 0);
    }*/



    // create text3d client info node LOCALLY
    Node* plyFltTextNode = scene_->CreateChild("Player Float Text", LOCAL);
    plyFltText_ = plyFltTextNode->CreateComponent<Text3D>(LOCAL);
    plyFltText_->SetColor(Color::GREEN);
    plyFltText_->SetEffectColor(Color::BLACK);
    plyFltText_->GetNode()->SetScale(40.0f);
    plyFltText_->SetFont(cache->GetResource<Font>(INGAME_FONT), 12);
    plyFltText_->SetFaceCameraMode(FC_ROTATE_XYZ);


    // Check when scene is rendered
    SubscribeToEvent(E_ENDRENDERING, URHO3D_HANDLER(MayaScape, HandleSceneRendered));
}

void MayaScape::UpdateUIState(bool state) {

    // Do the opposite for menu
    versionText_->SetVisible(!state);
    studioText_->SetVisible(!state);
   // instructionsText_->SetVisible(!state);
  //  hudText_->SetVisible(!state);
    buttonContainer_->SetVisible(!state);

    bkgSprite_->SetVisible(!state);

    // On client
    if (!isServer_) {

        // Create sprite and add to the UI layout
        powerBarP1Sprite_->SetVisible(state);
        powerBarBkgP1Sprite_->SetVisible(state);
        rpmBarP1Sprite_->SetVisible(state);
        rpmBarBkgP1Sprite_->SetVisible(state);
        velBarP1Sprite_->SetVisible(state);
        velBarBkgP1Sprite_->SetVisible(state);
        miniMapP1Sprite_->SetVisible(state);
        miniMapWPSprite_->SetVisible(state);
        miniMapBkgSprite_->SetVisible(state);
        markerMapBkgSprite_->SetVisible(state);
        steerWheelSprite_->SetVisible(state);


        // Create the UI for displaying the remaining lifes
//    lifeUI->SetVisible(false);
//    auto *lifeText = lifeUI->CreateChild<Text>("LifeText2");

        powerBarText_->SetVisible(state);
        rpmBarText_->SetVisible(state);
        velBarText_->SetVisible(state);


        // Debug text
        for (int i = 0; i < NUM_DEBUG_FIELDS; i++) {
            debugText_[i]->SetVisible(state);
        }

        packetsIn_->SetVisible(state);
        packetsOut_->SetVisible(state);

    } else {
        // On server

        // Debug text
        for (int i = 0; i < NUM_DEBUG_FIELDS; i++) {
            debugText_[i]->SetVisible(!state);
        }

        packetsIn_->SetVisible(!state);
        packetsOut_->SetVisible(!state);
    }
}

void MayaScape::HandleSceneRendered(StringHash eventType, VariantMap &eventData) {
    UnsubscribeFromEvent(E_ENDRENDERING);
    // Save the scene so we can reload it later
//    sample2D_->SaveScene(true);
    // Pause the scene as long as the UI is hiding it
    //   scene_->SetUpdateEnabled(false);
}

void MayaScape::HandleClientSceneLoaded(StringHash eventType, VariantMap& eventData)
{
    using namespace ClientSceneLoaded;
    URHO3D_LOGINFO("HandleClientSceneLoaded");
    URHO3D_LOGINFOF("Client: Scene checksum -> %s", ToStringHex(scene_->GetChecksum()).CString());

    // Client stores client object id
    auto *client = static_cast<Connection *>(eventData[P_CONNECTION].GetPtr());

    URHO3D_LOGINFOF("Client: Client info -> connected %d", client->IsConnected());

    // Clear existing replicated nodes on client
  //  scene_->Clear(true,false);

  scene_->MarkNetworkUpdate();


}

void MayaScape::SubscribeToEvents() {

    // Subscribe to UI element events
    SubscribeToEvent(textEdit_, E_TEXTFINISHED, URHO3D_HANDLER(MayaScape, HandleSend));
//    SubscribeToEvent(sendButton_, E_RELEASED, URHO3D_HANDLER(MayaScape, HandleSend));

    // Subscribe to log messages so that we can pipe them to the chat window
    SubscribeToEvent(E_LOGMESSAGE, URHO3D_HANDLER(MayaScape, HandleLogMessage));

    // Subscribe to network events
    SubscribeToEvent(E_NETWORKMESSAGE, URHO3D_HANDLER(MayaScape, HandleNetworkMessage));
    SubscribeToEvent(E_CLIENTSCENELOADED, URHO3D_HANDLER(MayaScape, HandleClientSceneLoaded));

    SubscribeToEvent(E_PHYSICSPRESTEP, URHO3D_HANDLER(MayaScape, HandlePhysicsPreStep));
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(MayaScape, HandlePostUpdate));

    // Subscribe to button actions
    SubscribeToEvent(playButton_, E_RELEASED, URHO3D_HANDLER(MayaScape, HandlePlayButton));
//    SubscribeToEvent(connectButton_, E_RELEASED, URHO3D_HANDLER(MayaScape, HandleConnect));
    SubscribeToEvent(disconnectButton_, E_RELEASED, URHO3D_HANDLER(MayaScape, HandleDisconnect));
    SubscribeToEvent(startServerButton_, E_RELEASED, URHO3D_HANDLER(MayaScape, HandleStartServer));
    SubscribeToEvent(exitButton_, E_RELEASED, URHO3D_HANDLER(MayaScape, HandleExit));

//    E_CONNECTFAILED
    SubscribeToEvent(E_CONNECTFAILED, URHO3D_HANDLER(MayaScape, HandleConnectionFailed));

    // Subscribe to server events
    SubscribeToEvent(E_SERVERSTATUS, URHO3D_HANDLER(MayaScape, HandleConnectionStatus));

    // Subscribe to player state events
    SubscribeToEvent(E_PLAYERSTATE, URHO3D_HANDLER(MayaScape, HandlePlayerStateUpdate));

    // Events sent between client & server (remote events) must be explicitly registered or else they are not allowed to be received
    GetSubsystem<Network>()->RegisterRemoteEvent(E_PLAYERSTATE);

    SubscribeToEvent(E_CLIENTOBJECTID, URHO3D_HANDLER(MayaScape, HandleClientObjectID));

    // Events sent between client & server (remote events) must be explicitly registered or else they are not allowed to be received
    GetSubsystem<Network>()->RegisterRemoteEvent(E_CLIENTOBJECTID);

    // Subscribe function for processing update events
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(MayaScape, HandleUpdate));

    // Subscribe HandlePostUpdate() function for processing post update events
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(MayaScape, HandlePostUpdate));

    // Subscribe to PostRenderUpdate to draw debug geometry
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(MayaScape, HandlePostRenderUpdate));

  //  SubscribeToEvent(E_JOYSTICKHATMOVE, URHO3D_HANDLER(MayaScape, HandleJoystickHatMove));
}


/*
void MayaScape::HandleNodeCollision(StringHash eventType, VariantMap& eventData) {

    using namespace PhysicsBeginContact2D;

    Node* p1Node = scene_->GetChild("Bear-P1", true);
    Node* p2Node = scene_->GetChild("Bear-P2", true);

    // Get colliding node
    auto* hitNodeA = static_cast<Node*>(eventData[PhysicsBeginContact2D::P_NODEA].GetPtr());
    auto* hitNodeB = static_cast<Node*>(eventData[PhysicsBeginContact2D::P_NODEB].GetPtr());

    URHO3D_LOGINFOF("hitNodeA=%d, hitNodeB=%d", hitNodeA, hitNodeB);
    URHO3D_LOGINFOF("hitNodeA id=%d, hitNodeB id=%d", hitNodeA->GetID(), hitNodeB->GetName());
        Vector2 contactPosition; 

        MemoryBuffer contacts(eventData[P_CONTACTS].GetBuffer());
        while (!contacts.IsEof()) {
            contactPosition = contacts.ReadVector2();
            auto contactNormal = contacts.ReadVector2();
            auto contactDistance = contacts.ReadFloat();
            auto contactImpulse = contacts.ReadFloat();
       //     std::cout << "contact position " << contactPosition.ToString().CString() << std::endl;
       //     std::cout << "contact normal " << contactNormal.ToString().CString() << std::endl;
       //     std::cout << "contact distance " << contactDistance << std::endl;
       //     std::cout << "contact impulse " << contactImpulse << std::endl;
        }

        std::cout << std::endl;

}*/

void MayaScape::HandlePlayerStateUpdate(StringHash eventType, VariantMap& eventData) {

    using namespace ClientPlayerState;
    int id = eventData[P_ID].GetUInt();
    int vehicleId = eventData[P_VEHICLE_ID].GetUInt();
    int life = eventData[P_LIFE].GetUInt();
    float rpm = eventData[P_RPM].GetFloat();
    float velocity = eventData[P_VELOCITY].GetFloat();
    float steer = eventData[P_STEER].GetFloat();

    // Store updated node id
    playerObjectID_ = id;
    playerVehicleID_ = vehicleId;

    URHO3D_LOGINFOF("Client -> HandlePlayerStateUpdate: %d, %d, %d, %f, %f, %f", id, vehicleId, life, rpm, velocity, steer);

}


void MayaScape::HandleCollisionBegin(StringHash eventType, VariantMap &eventData) {
    // Get colliding node
    auto *hitNode = static_cast<Node *>(eventData[PhysicsBeginContact2D::P_NODEA].GetPtr());
    if (hitNode->GetName() == "Bear-P1")
        hitNode = static_cast<Node *>(eventData[PhysicsBeginContact2D::P_NODEB].GetPtr());
    String nodeName = hitNode->GetName();
    Node *character2DNode = scene_->GetChild("Bear-P1", true);
}


void MayaScape::SetParticleEmitter(int hitId, float contactX, float contactY, int type, float timeStep) {
    // CREATE
    auto *cache = GetSubsystem<ResourceCache>();
    ParticleEffect2D *particleEffect;
    Vector2 position;

    switch (type) {
        case 0:
            particleEffect = cache->GetResource<ParticleEffect2D>("Urho2D/sun.pex");
            position.x_ = contactX;
            position.y_ = contactY;
            break;
        case 1:
            particleEffect = cache->GetResource<ParticleEffect2D>("Urho2D/power.pex");
            position.x_ = contactX;
            position.y_ = contactY;
            break;
    }

    if (!particleEffect)
        return;

    for (int i = 0; i < sizeof(particlePool_) / sizeof(*particlePool_); i++) {
        if (!particlePool_[i].used) {
            particlePool_[i].used = true;
            particlePool_[i].usedBy = hitId;
            particlePool_[i].node = scene_->CreateChild("GreenSpiral");
            auto *particleEmitter = particlePool_[i].node->CreateComponent<ParticleEmitter2D>(LOCAL);
            particleEmitter->SetEffect(particleEffect);
            particlePool_[i].node->SetPosition(Vector3(position.x_, position.y_, 0.0));
            particlePool_[i].lastEmit = timeStep;
            particlePool_[i].currEmit = 0;
            particlePool_[i].timeout = 0.8f;

            break;
        }
    }

    URHO3D_LOGINFOF("PARTICLE EMITTER CREATED used by id=%d", hitId);
}

void MayaScape::HandleUpdateParticlePool(float timeStep) {
    // CREATE
    auto *cache = GetSubsystem<ResourceCache>();
/*    auto* particleEffect = cache->GetResource<ParticleEffect2D>("Urho2D/sun.pex");
    if (!particleEffect)
        return;
*/
    for (int i = 0; i < sizeof(particlePool_) / sizeof(*particlePool_); i++) {
        if (particlePool_[i].used) {

            particlePool_[i].currEmit += timeStep;

            if (particlePool_[i].currEmit - particlePool_[i].lastEmit > particlePool_[i].timeout) {
                if (particlePool_[i].node) {
                    particlePool_[i].node->Remove();
                    particlePool_[i].used = false;
                    particlePool_[i].usedBy = -1;
                }
            }
        }
    }
}


void MayaScape::HandleCollisionEnd(StringHash eventType, VariantMap &eventData) {
    // Get colliding node
    auto *hitNode = static_cast<Node *>(eventData[PhysicsEndContact2D::P_NODEA].GetPtr());
    if (hitNode->GetName() == "Bear-P1")
        hitNode = static_cast<Node *>(eventData[PhysicsEndContact2D::P_NODEB].GetPtr());
    String nodeName = hitNode->GetName();
    Node *character2DNode = scene_->GetChild("Bear-P1", true);

}

void MayaScape::HandleRenderUpdate(StringHash eventType, VariantMap &eventData) {
    using namespace Update;
    auto *input = GetSubsystem<Input>();

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();
    HandleUpdateParticlePool(timeStep);

    float zoom_ = cameraNode_->GetComponent<Camera>()->GetZoom();
    float deltaSum;

    //    URHO3D_LOGINFOF("delta=%f", delta);
    //    URHO3D_LOGINFOF("factor=%f", factor);

    if (player_) {

        // **note** the buttons controls are handled in the character class update fn.
/*
        // right stick - camera
        Variant rStick = player_->controls_->extraData_[VAR_AXIS_1];

        if (!rStick.IsEmpty()) {
            Vector2 axisInput = rStick.GetVector2();
            player_->controls_->yaw_ += axisInput.x_ * YAW_SENSITIVITY;
            player_->controls_->pitch_ += axisInput.y_ * YAW_SENSITIVITY;
        }
*/
        // Limit pitch
        // player_->controls_.pitch_ = Clamp(player_->controls_.pitch_, -80.0f, 80.0f);
        // Set rotation already here so that it's updated every rendering frame instead of every physics frame
        //   player_->GetNode()->SetRotation(Quaternion(player_->controls_.yaw_, Vector3::UP));
        //  player_->GetNode()->SetRotation(Quaternion(0.0f, -180.0f-player_->heading_, 0.0f));
/*
        player_->GetNode()->SetRotation(Quaternion(0.0f, player_->heading_, 0.0));
        auto *model_ = player_->GetNode()->GetComponent<AnimatedModel>(true);
/*
//        model_->GetNode()->SetRotation(Quaternion(0, 90, 0));
        Skeleton &skeleton = model_->GetSkeleton();
        Bone *rootBone = skeleton.GetRootBone();
        Bone *startBone = rootBone;
*/


        /*
        // Clamp player life
        if (player_->life_ > 100) {
            player_->life_ = 100;
        }
        if (player_->life_ < 0) {
            player_->life_ = 0;
        }


        // Set animal scale
      //  player_->GetNode()->GetChildren()[0]->SetScale(20.0f);

        // AI
        if (agents_) {
            for (int i = 0; i < EvolutionManager::getInstance()->getAgents().size(); i++) {
                // Set rotation already here so that it's updated every rendering frame instead of every physics frame
                agents_[i]->GetNode()->SetRotation(Quaternion(agents_[i]->controls_.yaw_, Vector3::UP));
                //ai_[i]->GetNode()->SetRotation(Quaternion(0.0f, -180.0f-ai_[i]->heading_, 0.0f));
                //ai_[i]->GetNode()->SetRotation(Quaternion(-90.0f, ai_[i]->heading_+180.0f, 0.0f));
                agents_[i]->GetNode()->SetRotation(Quaternion(0.0f, agents_[i]->heading_, 0.0));

    /*
            static int _sndCnt = 0;
            float r = Random(-0.0f,5.0f);
            if (r > 2.5f) {
                _sndCnt++;
            }

            if (_sndCnt > 5) {
                sample2D_->PlaySoundEffect("enemy01-laugh.wav");
            }


                // Update billboards (genotype, powerbar)

                const float BILLBOARD_ROTATION_SPEED = 50.0f;

                // Genotype

                // Rotate the individual billboards within the billboard sets, then recommit to make the changes visible
                for (unsigned j = 0; j < agents_[i]->genotypeBBSet_->GetNumBillboards(); ++j) {
                    Billboard *bb = agents_[i]->genotypeBBSet_->GetBillboard(j);
                    //  bb->rotation_ += BILLBOARD_ROTATION_SPEED * timeStep;
                    if (agents_[i]) {
                        Vector3 aiPos = agents_[i]->GetNode()->GetPosition();
                        bb->position_ = Vector3(aiPos.x_ + (j * 0.02), aiPos.y_, 0.0f);

                        // Already set size in initialization (based on parameter)
                        //bb->size_ = Vector2((1.0f) * 0.05f, (0.1f) * 0.05f);


                        //       bb->position_ = Vector3(player_->GetNode()->GetPosition().x_, player_->GetNode()->GetPosition().y_, -5.0f);
                    }

                }

                agents_[i]->genotypeBBSet_->Commit();


                // Powerbar

                // Rotate the individual billboards within the billboard sets, then recommit to make the changes visible
                for (unsigned j = 0; j < agents_[i]->powerbarBBSet_->GetNumBillboards(); ++j) {
                    Billboard *bb = agents_[i]->powerbarBBSet_->GetBillboard(j);
                    //  bb->rotation_ += BILLBOARD_ROTATION_SPEED * timeStep;
                    if (agents_[i]) {
                        Vector3 aiPos = agents_[i]->GetNode()->GetPosition();
                        bb->position_ = Vector3(aiPos.x_ + (j * 0.02), aiPos.y_+0.2f, 0.0f);

                        bb->size_ = Vector2((0.4f) * 0.05f, (4.0f) * 0.05f);

                        //       bb->position_ = Vector3(player_->GetNode()->GetPosition().x_, player_->GetNode()->GetPosition().y_, -5.0f);
                    }

                }

                agents_[i]->powerbarBBSet_->Commit();

            }
        }

        player_->life_ = 50;
         */

    }
    Variant lStick = ntwkControls_.extraData_[VAR_AXIS_0];
    Vector2 lAxisVal = lStick.GetVector2();
    float steering = lStick.GetVector2().x_ * 0.25f;
    //    Quaternion vRot = vehicle_->GetNode()->GetRotation();

    steerWheelSprite_->SetRotation(360.0f * steering);


    int life = 100;
    if (player_) {
        life = 0;//player_->GetLife();
        // Update player powerbar
        IntVector2 v = powerBarBkgP1Sprite_->GetSize();
        int power = int(((life) / 100.0f) * (float) v.x_);
        powerBarP1Sprite_->SetSize(power, v.y_);

        float maxRPM = 8000.0f;
        float clampedRPM = 0;//player_->GetVehicle()->GetCurrentRPM();
        if (clampedRPM > maxRPM) {
            clampedRPM = maxRPM;
        }

        v = rpmBarBkgP1Sprite_->GetSize();
        int rpm = ((clampedRPM / maxRPM) * v.x_);
        rpmBarP1Sprite_->SetSize(rpm, v.y_);


        float maxSpeed = 220.0f;
        float clampedSpeed = 0;//player_->GetVehicle()->GetSpeedKmH();
        if (clampedSpeed > maxSpeed) {
            clampedSpeed = maxSpeed;
        }

        // Cap speed at 220 KmH
        v = velBarBkgP1Sprite_->GetSize();
        int velocity = ((clampedSpeed / maxSpeed) * v.x_);
        velBarP1Sprite_->SetSize(velocity, v.y_);

//    float maxX = terrain_->GetPatchSize()*terrain_->GetNumPatches().x_;
//    float maxY = terrain_->GetPatchSize()*terrain_->GetNumPatches().y_;
        //
        // Position
        //Vector3 position((float)x * spacing_.x_, GetRawHeight(xPos, zPos), (float)z * spacing_.z_);


        //    if (isServer_) {


//    float maxX = terrain_->GetHeightMap()->GetWidth()*terrain_->GetPatchSize();
//    float maxY = terrain_->GetHeightMap()->GetHeight()*terrain_->GetPatchSize();



        // Only show once vehicle is activated
        if (player_) {
            // Calculate mini map position
            Vector3 shiftedRange =
                    player_->GetPosition() + Vector3(mapSize / 2, mapSize / 2, mapSize / 2);

            // 1600+1600
            float xRange = (shiftedRange.x_ / mapSize) * miniMapWidth;
            float zRange = (shiftedRange.z_ / mapSize) * miniMapHeight;

            float miniMapP1X = miniMapBkgSprite_->GetPosition().x_;
            float miniMapP1Y = miniMapBkgSprite_->GetPosition().y_;

            // Update mini map for P1 position
            //    miniMapP1Sprite_->SetPosition(Vector2(776.0f-16.0f, 300.0f));
            float startRotOffset = 180.0f;
            miniMapP1Sprite_->SetPosition(Vector2(miniMapP1X - xRange + 0.0f, miniMapP1Y - zRange + 0.0f));
            miniMapP1Sprite_->SetRotation(vehicleRot_.YawAngle() + startRotOffset);

            float wpOffsetX = -mapSize / 2;
            float wpOffsetY = -mapSize / 2;

            int index = 0;//player_->wpActiveIndex_;

            if (index < 0) index = 0;
            // Convert marker position to world position for waypoint
            float wpPosX =
                    (((float) waypoints_[index].x_ / (float) terrain_->GetMarkerMap()->GetWidth()) * mapSize) +
                    wpOffsetX;
            float wpPosZ =
                    (((float) waypoints_[index].z_ / (float) terrain_->GetMarkerMap()->GetHeight()) * mapSize) +
                    wpOffsetY;

            // Calculate mini map position for waypoint
            shiftedRange = Vector3(wpPosX, 0.0f, wpPosZ) + Vector3(mapSize / 2, mapSize / 2, mapSize / 2);

            // 1600+1600
            xRange = (shiftedRange.x_ / mapSize) * miniMapWidth;
            zRange = (shiftedRange.z_ / mapSize) * miniMapHeight;

            float miniMapWPX = miniMapBkgSprite_->GetPosition().x_;
            float miniMapWPY = miniMapBkgSprite_->GetPosition().y_;


            // Update mini map for WP position
            //    miniMapP1Sprite_->SetPosition(Vector2(776.0f-16.0f, 300.0f));
            miniMapWPSprite_->SetPosition(Vector2(miniMapWPX - xRange, miniMapWPY - zRange));
            //    miniMapWPSprite_->SetRotation(vehicleRot_.YawAngle());




            /* DISABLED HEADLAMP RENABLE AFTER UPDATE
             player_->GetVehicleHeadLamp()->SetPosition(Vector3(player_->GetVehicle()->GetNode()->GetPosition().x_,
                                                                player_->GetVehicle()->GetNode()->GetPosition().y_ + 5.0f,
                                                                player_->GetVehicle()->GetNode()->GetPosition().z_));
             player_->GetVehicleHeadLamp()->SetDirection(Vector3(player_->GetVehicle()->GetNode()->GetRotation().x_, 1.0f,
                                                                 player_->GetVehicle()->GetNode()->GetRotation().z_));


             // Update vehicle head lamp lighting
             Light *light = player_->GetVehicleHeadLamp()->GetComponent<Light>();
             // Light* light = vehicleHeadLamp_->CreateComponent<Light>();
             //light->SetLightType(LIGHT_SPOT)
             light->SetLightType(LIGHT_POINT);
             light->SetRange(30.0f);

             float rpmLightFactor = 40.0f;
             float rpmLight = rpm / 4500.0f;
             if (rpmLight > 0.8f) rpmLight = 0.8f;
             light->SetBrightness(0.2f + (rpmLight * rpmLightFactor));
             light->SetCastShadows(true);
             //   light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
     */
            // Update focus objects
//            focusObjects_[0] = player_->GetVehicle()->GetNode()->GetPosition(); // Vehicle
        }

        int i = 0;

        //URHO3D_LOGINFOF("player_ position x=%f, y=%f, z=%f", player_->GetNode()->GetPosition().x_, player_->GetNode()->GetPosition().y_, player_->GetNode()->GetPosition().z_);

        //
    }


    // stat
#ifdef SHOW_STATS
    framesCount_++;
    if ( fpsTimer_.GetMSec(false) >= ONE_SEC_DURATION )
    {
        Renderer *renderer = GetSubsystem<Renderer>();
        String stat;

        stat.AppendWithFormat( "tris: %d fps: %d",
                               renderer->GetNumPrimitives(),
                               framesCount_);

#ifdef SHOW_CAM_POS
        String x, y, z;
        char buff[20];
        sprintf(buff, ", cam: %.1f, ", cameraNode_->GetPosition().x_);
        x = String(buff);
        sprintf(buff, "%.1f, ", cameraNode_->GetPosition().y_);
        y = String(buff);
        sprintf(buff, "%.1f", cameraNode_->GetPosition().z_);
        z = String(buff);
        stat += x + y + z;
#endif

        textStatus_->SetText(stat);
        framesCount_ = 0;
        fpsTimer_.Reset();
    }
#endif

}

void MayaScape::HandleUpdate(StringHash eventType, VariantMap &eventData) {
    using namespace Update;
    auto *input = GetSubsystem<Input>();

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();
    HandleUpdateParticlePool(timeStep);

///    bkgSprite_->SetScale

    bkgAngle_ += timeStep * 2.0f;

    if (bkgAngle_ > 360.0f) {
        bkgAngle_ = 0.0f;
    }
    // Update menu background
    bkgSprite_->SetRotation(bkgAngle_);
    auto *cache = GetSubsystem<ResourceCache>();

    UI *ui = GetSubsystem<UI>();
    Server *server = GetSubsystem<Server>();

    Vector<Urho3D::String> loginList;

    // On the client, use the lcoal login list
    if (isServer_) {
        HashMap<String, Connection *> map = server->GetLoginList();
        loginList = map.Keys();
    } else {
        loginList = loginList_;
    }

/*
    std::string playerInfo;
    char str[40];

    Vector3 pos = player_->GetVehicle()->GetNode()->GetPosition();
    sprintf(str, "%f,%f,%f", pos.x_, pos.y_, pos.z_);
    playerInfo.clear();
    playerInfo.append("Vehicle position (x,y,z) -> ").append(str);
    //  debugText_[i]->SetText(playerInfo.c_str());
*/

    // Login List
    if (isServer_) {
/* DISABLE ON-SCREEN LOGIN LIST - will show through console
        if (loginList.Size() > 0) {
            // Show login list
            debugText_[0]->SetText("Server: Client List");
            debugText_[0]->SetAlignment(HA_LEFT, VA_TOP);
            debugText_[0]->SetPosition(10.0f, 400);
            debugText_[0]->SetVisible(true);
            for (int i = 1; i < loginList.Size() + 1; i++) {

                if (loginList.Size() < NUM_DEBUG_FIELDS) {
                    debugText_[i]->SetAlignment(HA_LEFT, VA_TOP);
                    debugText_[i]->SetPosition(10.0f, 400 + (i * 20));
                    debugText_[i]->SetVisible(true);
                    //          std::string debugData1;
//            debugData1.append("-");
                    debugText_[i]->SetText(loginList[i - 1].CString());
                }
            }
        }*/

    } else {
        if (loginList.Size() > 0) {
            // Show login list
            debugText_[0]->SetText("Client: Connection List");
            debugText_[0]->SetAlignment(HA_LEFT, VA_TOP);
            debugText_[0]->SetPosition(10.0f, 400);
            debugText_[0]->SetVisible(true);
            for (int i = 1; i < loginList.Size() + 1; i++) {

                if (loginList.Size() < NUM_DEBUG_FIELDS) {
                    debugText_[i]->SetAlignment(HA_LEFT, VA_TOP);
                    debugText_[i]->SetPosition(10.0f, 400 + (i * 20));
                    debugText_[i]->SetVisible(true);
                    //          std::string debugData1;
//            debugData1.append("-");
                    debugText_[i]->SetText(loginList[i - 1].CString());
                }
            }
        }
    }

    GameController *gameController = GetSubsystem<GameController>();
    gameController->UpdateControlInputs(ntwkControls_);

    float deltaSum;

    if (player_) {

//        Vector3 targetAgent = agents_[player_->targetAgentIndex_]->GetVehicle()->GetNode()->GetPosition();


        // DISABLE AGENT TARGETING FOR NOW
        /*
            btTransform trans;
            agents_[player_->targetAgentIndex_]->vehicle_->GetRaycastVehicle()->getWorldTransform(trans);
            Vector3 posWS = ToVector3(trans.getOrigin());
            Vector3 targetAgent = posWS;
            player_->SetTarget(targetAgent);

    */

//        gameController->UpdateControlInputs(player_->GetVehicle()->controls_);



        /*
            // Determine zoom by getting average distance from all players
            for (int i = 0; i < EvolutionManager::getInstance()->getAgents().size(); i++) {

                // Update player location for AI
                agents_[i]->playerPos_ = player_->GetNode()->GetPosition();

                Vector3 p1 = player_->GetNode()->GetPosition();
                p1.z_ = 0;
                Vector3 p2 = agents_[i]->GetNode()->GetPosition();
                p2.z_ = 0;
                float delta = p1.DistanceToPoint(p2);
                deltaSum += delta;
            }

            float avgDelta = ((float) deltaSum) / ((float) EvolutionManager::getInstance()->getAgents().size());
            float factor;

            if (avgDelta > 5.0f) {
                factor = 1.0f - avgDelta * 0.02f;
            } else {
                factor = 1.0f + avgDelta * 0.02f;
            }

            factor = 1.0f;

            zoom_ = Clamp(zoom_ * factor, CAMERA_MIN_DIST, CAMERA_MAX_DIST);
            cameraNode_->GetComponent<Camera>()->SetZoom(zoom_);

       */
    }

    //    URHO3D_LOGINFOF("delta=%f", delta);
    //    URHO3D_LOGINFOF("factor=%f", factor);


    // Toggle debug geometry with 'Z' key
    if (input->GetKeyPress(KEY_Z))
        drawDebug_ = !drawDebug_;

    if (player_) {
        /*
        // Toggle debug geometry with 'C' key
        player_->changeTargetTime_ += timeStep;

        if ((player_->GetVehicle()->controls_.IsDown(BUTTON_Y)) || (input->GetKeyPress(KEY_C))) {
            if (player_->changeTargetTime_ > 0.04f)
                player_->targetAgentIndex_++;
            player_->targetAgentIndex_ =
                    player_->targetAgentIndex_ % EvolutionManager::getInstance()->getAgents().size();
            player_->changeTargetTime_ = 0;
        }

        if (input->GetKeyPress(KEY_U)) {
            player_->autoSteering_ = !player_->autoSteering_;
        }

        if (input->GetKeyPress(KEY_R)) {
            // Place on track
            player_->GetVehicle()->GetNode()->SetPosition(
                    Vector3(raceTrack_->GetPosition().x_ + Random(-mapSize / 4, mapSize / 4), 500.0f,
                            raceTrack_->GetPosition().z_ + Random(-mapSize / 4, mapSize / 4)));
        }

        if (input->GetKeyPress(KEY_O)) {

            // Set focus to vehicle
            focusIndex_ = 0;

            // 453 -202
            player_->GetVehicle()->GetNode()->SetPosition(Vector3(453.0f, 500.0f, -202.0f));

            // Place on track origin
//        vehicle_->GetNode()->SetPosition(Vector3(raceTrack_->GetPosition().x_, 500.0f, raceTrack_->GetPosition().z_));


            // TOP LEFT EDGE
//        vehicle_->GetNode()->SetPosition(Vector3(mapSize/2, 200.0f, mapSize/2));

            // BOTTOM RIGHT EDGE
//        vehicle_->GetNode()->SetPosition(Vector3(-mapSize/2, 200.0f, -mapSize/2));

        }
//    player_->GetVehicle()->controls_
        if (player_->GetVehicle()->controls_.IsDown(BUTTON_X)) {

            if (player_->GetLastFire() > WaitTimeNextFire) {
                player_->SetLastFire(0);

                float wpOffsetX = -mapSize / 2;
                float wpOffsetY = -mapSize / 2;
                // Convert marker position to world position for waypoint
                float wpPosX =
                        (((float) waypoints_[player_->wpActiveIndex_].x_ /
                          (float) terrain_->GetMarkerMap()->GetWidth()) *
                         mapSize) + wpOffsetX;
                float wpPosZ =
                        (((float) waypoints_[player_->wpActiveIndex_].z_ /
                          (float) terrain_->GetMarkerMap()->GetHeight()) *
                         mapSize) + wpOffsetY;

                URHO3D_LOGINFOF("Fire=%f", player_->GetLastFire());

                player_->Fire();
            } else {
                URHO3D_LOGINFOF("Waiting last fire=%f", player_->GetLastFire());
            }

        }

        if (input->GetKeyPress(KEY_R)) {
            player_->wpActiveIndex_ = 0;
        }

        if (input->GetKeyPress(KEY_N)) {
            player_->wpActiveIndex_++;

            player_->wpActiveIndex_ = player_->wpActiveIndex_ % waypoints_.Size();
            // Place on track origin
//        vehicle_->GetNode()->SetPosition(Vector3(raceTrack_->GetPosition().x_, 500.0f, raceTrack_->GetPosition().z_));
        }
        */
    }

    // Toggle through focus objects
    if (input->GetKeyPress(KEY_T)) {

        //1448.78039550781,0,1507.31701660156
        //Vector3 trackMarkerPos = Vector3(724.390197753906,0,753.658508300781);
        // Place on at focus object
//        vehicle_->GetNode()->SetPosition(Vector3(focusObjects_[focusIndex_].x_, 500.0f, focusObjects_[focusIndex_].z_));

        // Increment focus object
        focusIndex_++;
    }

    if (!focusObjects_.Empty()) {
        // Clamp focus index
        focusIndex_ = focusIndex_ % focusObjects_.Size();
    }



    if (input->GetKeyPress(KEY_F7))
        ReloadScene(false);


    ui = GetSubsystem<UI>();

    int life = 100;
    if (player_) {
        life = 0;//player_->GetLife();

/*
            // qualify vehicle orientation
            Vector3 v3Up = vehicleNode->GetWorldUp();
            float fUp = v3Up.DotProduct(Vector3::UP);

            if (v3Up.y_ < 0.1f) {
                // maintain its orientation
                Vector3 vPos = vehicleNode->GetWorldPosition();
                Vector3 vForward = player_->GetVehicle()->GetNode()->GetDirection();
                Quaternion qRot;
                qRot.FromLookRotation(vForward);

                vPos += Vector3::UP * 3.0f;
                vehicleNode->SetTransform(vPos, qRot);
                player_->GetVehicle()->ResetForces();
            }

            */
    }


    // Toggle physics debug geometry with space
    if (input->GetKeyPress(KEY_F5)) {
//        drawDebug_ = !drawDebug_;
        SaveScene(false);
    }


    // Check for loading / saving the scene
    if (input->GetKeyPress(KEY_K)) {
        File saveFile(context_, GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Scenes/MayaScape_Output.xml",
                      FILE_WRITE);
        scene_->SaveXML(saveFile);
    }

    /*
    // updating half the boids at a time depending on the update cycle index
    if (updateCycleIndex == 0) {
        for (int i = 0; i < (numOfBoidsets / 2); i++) {
            boids[i].Update(timeStep);
        }
        updateCycleIndex = 1;
    } else if (updateCycleIndex == 1) {
        for (int i = (numOfBoidsets / 2); i < numOfBoidsets; i++) {
            boids[i].Update(timeStep);
        }
        updateCycleIndex = 0;
    }*/



    // stat
#ifdef SHOW_STATS
    framesCount_++;
    if ( fpsTimer_.GetMSec(false) >= ONE_SEC_DURATION )
    {
        Renderer *renderer = GetSubsystem<Renderer>();
        String stat;

        stat.AppendWithFormat( "tris: %d fps: %d",
                               renderer->GetNumPrimitives(),
                               framesCount_);

#ifdef SHOW_CAM_POS
        String x, y, z;
        char buff[20];
        sprintf(buff, ", cam: %.1f, ", cameraNode_->GetPosition().x_);
        x = String(buff);
        sprintf(buff, "%.1f, ", cameraNode_->GetPosition().y_);
        y = String(buff);
        sprintf(buff, "%.1f", cameraNode_->GetPosition().z_);
        z = String(buff);
        stat += x + y + z;
#endif

        textStatus_->SetText(stat);
        framesCount_ = 0;
        fpsTimer_.Reset();
    }
#endif

    // Call our render update
    HandleRenderUpdate(eventType, eventData);
}

void MayaScape::HandlePostUpdate(StringHash eventType, VariantMap &eventData) {

    if (packetCounterTimer_.GetMSec(false) > 1000 && GetSubsystem<Network>()->GetServerConnection()) {
        packetsIn_->SetText(
                "Packets  in: " + String(GetSubsystem<Network>()->GetServerConnection()->GetPacketsInPerSec()));
        packetsOut_->SetText(
                "Packets out: " + String(GetSubsystem<Network>()->GetServerConnection()->GetPacketsOutPerSec()));
        packetCounterTimer_.Reset();
    }
    if (packetCounterTimer_.GetMSec(false) > 1000 && GetSubsystem<Network>()->GetClientConnections().Size()) {
        int packetsIn = 0;
        int packetsOut = 0;
        auto connections = GetSubsystem<Network>()->GetClientConnections();
        for (auto it = connections.Begin(); it != connections.End(); ++it) {
            packetsIn += (*it)->GetPacketsInPerSec();
            packetsOut += (*it)->GetPacketsOutPerSec();
        }
        packetsIn_->SetText("Packets  in: " + String(packetsIn));
        packetsOut_->SetText("Packets out: " + String(packetsOut));
        packetCounterTimer_.Reset();
    }

    if (started_) {

        if (isServer_) {
            // SERVER
            // Set server cam to aerial
            SetAerialCamera();
        } else {
            // CLIENT

            Node *actorNode = nullptr;
            actorNode = scene_->GetNode(playerObjectID_);

            using namespace Update;
            float timeStep = eventData[P_TIMESTEP].GetFloat();

            if (actorNode) {
                // Update player node
                player_ = static_cast<SharedPtr<Node>>(actorNode);

                // Apply transformations to camera
                MoveCamera(actorNode, timeStep);


            }

            instructionsText_->SetVisible(true);

            for (int i = 0; i < hudTextList_.Size(); i++) {
                hudTextList_[i]->SetVisible(true);
            }
        }
    }

}

void MayaScape::HandlePostRenderUpdate(StringHash eventType, VariantMap &eventData) {

    if (player_) {

        if (drawDebug_) {
            // bones. Note that debug geometry has to be separately requested each frame. Disable depth test so that we can see the
            // bones properly
            GetSubsystem<Renderer>()->DrawDebugGeometry(false);

        }
    }

    if (doSpecial_) {
        //   player_->animCtrl_->GetTime(WA)
    }

}

void MayaScape::HandleJoystickHatMove(StringHash eventType, VariantMap &eventData) {

    using namespace JoystickHatMove;
    int joystickID = eventData[P_JOYSTICKID].GetInt();
    int hat = eventData[P_HAT].GetInt();
    int value = eventData[P_POSITION].GetInt();

    // Clear network control bits
    ntwkControls_.Set(NTWK_CTRL_FORWARD, 0);
    ntwkControls_.Set(NTWK_CTRL_LEFT, 0);
    ntwkControls_.Set(NTWK_CTRL_RIGHT, 0);
    ntwkControls_.Set(NTWK_CTRL_BACK, 0);

    switch (value) {
        case 1: // up
            ntwkControls_.Set(NTWK_CTRL_FORWARD, 1);
            break;

        case 2: // right
            ntwkControls_.Set(NTWK_CTRL_RIGHT, 1);
        break;

        case 8: // left
            ntwkControls_.Set(NTWK_CTRL_LEFT, 1);
            break;

        case 4: // down
            ntwkControls_.Set(NTWK_CTRL_BACK, 1);
            break;

    }
}


void MayaScape::ReloadScene(bool reInit) {
/*
    String filename = sample2D_->demoFilename_;
    if (!reInit)
        filename += "InGame";

    File loadFile(context_, GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Scenes/" + filename + ".xml",
                  FILE_READ);
    scene_->LoadXML(loadFile);*/

}

void MayaScape::PlaySoundEffect(const String &soundName) {

    /*
     *
    // loading the sound
    sound_flag = cache->GetResource<Sound>("Music/Sad_0.wav");
    sound_source_flag = modelNode->CreateComponent<SoundSource3D>();
    sound_source_flag->SetNearDistance(1);  // distance up to where the volume is 100%
    sound_source_flag->SetFarDistance(1550);  // distance from where the volume is at 0%
    sound_source_flag->SetSoundType(SOUND_EFFECT);

    sound_source_flag->Play(sound_flag);
*/



    auto *cache = GetSubsystem<ResourceCache>();
//    auto* source = scene_->CreateComponent<SoundSource>();
    auto *source = scene_->CreateComponent<SoundSource3D>(LOCAL);


    // loading the sound
//    sound_flag = cache->GetResource<Sound>("Music/test.wav");
    //sound_source_flag = modelNode->CreateComponent<SoundSource3D>();

    source->SetNearDistance(1);  // distance up to where the volume is 100%
    source->SetFarDistance(6000);  // distance from where the volume is at 0%
    source->SetSoundType(SOUND_MUSIC);

    auto *sound = cache->GetResource<Sound>("Sounds/" + soundName);
    if (sound != nullptr) {
        source->SetAutoRemoveMode(REMOVE_COMPONENT);
        source->Play(sound);
    }
}


void MayaScape::HandlePlayButton(StringHash eventType, VariantMap &eventData) {

    PlaySoundEffect("BAY-r1-mono.wav");

    // Remove fullscreen UI and unfreeze the scene
    auto *ui = GetSubsystem<UI>();
    if (static_cast<Text *>(ui->GetRoot()->GetChild("FullUI", true))) {
        ui->GetRoot()->GetChild("FullUI", true)->Remove();
        scene_->SetUpdateEnabled(true);
    } else
        // Reload scene
        ReloadScene(true);

    // Hide mouse cursor
    auto *input = GetSubsystem<Input>();
    input->SetMouseVisible(false);

    // Call on connect to server
    HandleConnect(eventType, eventData);
}

// Network functions
void MayaScape::CreateServerSubsystem() {
    context_->RegisterSubsystem(new Server(context_));
}

void MayaScape::CreateAdminPlayer() {
/*
    Node* clientNode = scene_->CreateChild("Admin");
    clientNode->SetPosition(Vector3(Random(40.0f) - 20.0f, 100.0f, Random(40.0f) - 20.0f));

    ClientObj *clientObj = (ClientObj*)clientNode->CreateComponent(NetworkActor::GetTypeStatic(), REPLICATED);

    ((NetworkActor*)clientObj)->isServer_ = isServer_;
    // set identity
    clientObj->SetClientInfo("ADMIN", 99);
    clientObjectID_ = clientNode->GetID();*/
    isServer_ = true;

}

void MayaScape::CreateUI() {
    ResourceCache *cache = GetSubsystem<ResourceCache>();
    UI *ui = GetSubsystem<UI>();
    UIElement *root = ui->GetRoot();
    XMLFile *uiStyle = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");
    root->SetDefaultStyle(uiStyle);


    SharedPtr<Cursor> cursor(new Cursor(context_));
    cursor->SetStyleAuto(uiStyle);
    ui->SetCursor(cursor);
    Graphics *graphics = GetSubsystem<Graphics>();
    cursor->SetPosition(graphics->GetWidth() / 2, graphics->GetHeight() / 2);


    // Create the UI content
//    sample2D_->CreateUIContent("MayaScape v0.1");
//    auto* ui = GetSubsystem<UI>();
    //   Button *playButton = static_cast<Button *>(ui->GetRoot()->GetChild("PlayButton", true));
//    SubscribeToEvent(playButton, E_RELEASED, URHO3D_HANDLER(MayaScape, HandlePlayButton));

    int textureWidth;
    int textureHeight;

    // Get logo texture
    Texture2D *bkgTexture = cache->GetResource<Texture2D>("Textures/menu-bkg.png");
    if (!bkgTexture)
        return;

    // Create bkg sprite and add to the UI layout
    bkgSprite_ = ui->GetRoot()->CreateChild<Sprite>();
    bkgAngle_ = 0.0f;

    // Set logo sprite texture
    bkgSprite_->SetTexture(bkgTexture);

    textureWidth = bkgTexture->GetWidth();
    textureHeight = bkgTexture->GetHeight();

    // Set logo sprite scale
    bkgSprite_->SetScale((256.0f / textureWidth) * 9.2f);

    // Set logo sprite size
    bkgSprite_->SetSize(textureWidth, textureHeight);

    // Set logo sprite hot spot
    bkgSprite_->SetHotSpot(textureWidth / 2, textureHeight / 2);

    // Set logo sprite alignment
    bkgSprite_->SetAlignment(HA_CENTER, VA_CENTER);
    bkgSprite_->SetPosition(0, 0);

    // Make logo not fully opaque to show the scene underneath
    bkgSprite_->SetOpacity(0.9f);

    // Set a low priority for the logo so that other UI elements can be drawn on top
    bkgSprite_->SetPriority(-100);

    // Construct the instructions text element
    versionText_ = ui->GetRoot()->CreateChild<Text>();
    versionText_->SetText(APP_VERSION);
    versionText_->SetFont(cache->GetResource<Font>("Fonts/CompassGold.ttf"), 120);
    versionText_->SetColor(Color::WHITE);
    // Position the text relative to the screen center
    versionText_->SetHorizontalAlignment(HA_CENTER);
    versionText_->SetPosition(15, 40);
    // Hide once connected
    versionText_->SetVisible(true);

    studioText_ = ui->GetRoot()->CreateChild<Text>();
    studioText_->SetText(STUDIO_VERSION);
    studioText_->SetFont(cache->GetResource<Font>("Fonts/CompassGold.ttf"), 26);
    studioText_->SetColor(Color::BLACK);
    // Position the text relative to the screen center
    studioText_->SetHorizontalAlignment(HA_CENTER);
    studioText_->SetPosition(0, 920);
    // Hide once connected
    studioText_->SetVisible(true);



    // Construct the instructions text element
    instructionsText_ = ui->GetRoot()->CreateChild<Text>();
    instructionsText_->SetText(
            ""
            "An online open world combat racing game."
    );
    instructionsText_->SetFont(cache->GetResource<Font>(INGAME_FONT2), 24);
    instructionsText_->SetColor(Color::WHITE);
    // Position the text relative to the screen center
    instructionsText_->SetHorizontalAlignment(HA_CENTER);
    instructionsText_->SetPosition(0, 550);
    // Hide until connected
    instructionsText_->SetVisible(false);


    int hudTextCount = 4;
    for (int i = 0; i < hudTextCount; i++) {
        // Construct the text element
        SharedPtr<Text> hudText_ = static_cast<SharedPtr<Text>>(ui->GetRoot()->CreateChild<Text>());
        hudText_->SetText("");
        hudText_->SetFont(cache->GetResource<Font>(INGAME_FONT2), 17);
        hudText_->SetColor(Color::WHITE);
        // Position the text relative to the screen center
        hudText_->SetHorizontalAlignment(HA_CENTER);
        hudText_->SetPosition(0, 610+(i*20.0f));
        // Hide until connected
        hudText_->SetVisible(false);

        hudTextList_.Push(hudText_);
    }

    buttonContainer_ = root->CreateChild<UIElement>();
    buttonContainer_->SetFixedSize(1800, 600);
    buttonContainer_->SetPosition(490, 300);
    buttonContainer_->SetHorizontalAlignment((HA_CENTER));
    buttonContainer_->SetLayoutMode(LM_VERTICAL);
    buttonContainer_->SetLayoutSpacing(10.0);
    textEdit_ = buttonContainer_->CreateChild<LineEdit>();
    textEdit_->SetStyleAuto();
    textEdit_->SetVisible(false);

    playButton_ = CreateButton("Play", 800);
    disconnectButton_ = CreateButton("Disconnect", 800);
    startServerButton_ = CreateButton("Start Server", 800);
    exitButton_ = CreateButton("Exit", 800);

    // Get logo texture
    Texture2D *logoTexture = cache->GetResource<Texture2D>("Textures/logo.png");
    if (!logoTexture)
        return;

    // Create logo sprite and add to the UI layout
    //logoSprite_ = ui->GetRoot()->CreateChild<Sprite>();

    // Set logo sprite texture
    //logoSprite_->SetTexture(logoTexture);

    textureWidth = logoTexture->GetWidth();
    textureHeight = logoTexture->GetHeight();

    // Set logo sprite scale
    logoSprite_->SetScale(256.0f / textureWidth);

    // Set logo sprite size
    logoSprite_->SetSize(textureWidth, textureHeight);

    // Set logo sprite hot spot
    logoSprite_->SetHotSpot(textureWidth, textureHeight);

    // Set logo sprite alignment
    logoSprite_->SetAlignment(HA_CENTER, VA_TOP);
    logoSprite_->SetPosition(130, 300);

    // Make logo not fully opaque to show the scene underneath
    logoSprite_->SetOpacity(0.9f);

    // Set a low priority for the logo so that other UI elements can be drawn on top
    logoSprite_->SetPriority(-100);

    auto *font = cache->GetResource<Font>(INGAME_FONT2);
    chatHistoryText_ = root->CreateChild<Text>();
    chatHistoryText_->SetFont(font, 12);
    chatHistoryText_->SetVisible(false);

    UpdateButtons();

    float rowHeight = chatHistoryText_->GetRowHeight();
    // Row height would be zero if the font failed to load
    if (rowHeight) {
        float numberOfRows = (graphics->GetHeight() - 100) / rowHeight;
        chatHistory_.Resize(static_cast<unsigned int>(numberOfRows));
    }

    // No viewports or scene is defined. However, the default zone's fog color controls the fill color
    GetSubsystem<Renderer>()->GetDefaultZone()->SetFogColor(Color(0.0f, 0.0f, 0.1f));
}

void MayaScape::SetupViewport() {
    Renderer *renderer = GetSubsystem<Renderer>();

//    GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, camera));

    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void MayaScape::ChangeDebugHudText() {
    // change profiler text
    if (GetSubsystem<DebugHud>()) {
        Text *dbgText = GetSubsystem<DebugHud>()->GetProfilerText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetStatsText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetMemoryText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);

        dbgText = GetSubsystem<DebugHud>()->GetModeText();
        dbgText->SetColor(Color::CYAN);
        dbgText->SetTextEffect(TE_NONE);
    }
}

Button *MayaScape::CreateButton(const String &text, int width) {
    ResourceCache *cache = GetSubsystem<ResourceCache>();
    Font *font = cache->GetResource<Font>("Fonts/CompassGold.ttf");

    Button *button = buttonContainer_->CreateChild<Button>();
    button->SetStyleAuto();
    button->SetFixedWidth(width);

    Text *buttonText = button->CreateChild<Text>();
    buttonText->SetName("text");
    buttonText->SetFont(font, 48);
    buttonText->SetAlignment(HA_CENTER, VA_CENTER);
    buttonText->SetText(text);

    return button;
}

void MayaScape::UpdateButtons() {
    Network *network = GetSubsystem<Network>();
    Connection *serverConnection = network->GetServerConnection();
    bool serverRunning = network->IsServerRunning();


    playButton_->SetVisible(!serverConnection && !serverRunning);
    // Show and hide buttons so that eg. Connect and Disconnect are never shown at the same time
    disconnectButton_->SetVisible(serverConnection || serverRunning);
    Text *discText = disconnectButton_->GetChildStaticCast<Text>(String("text"));
    if (serverConnection) {
        discText->SetText("Client Disconnect");
    } else if (serverRunning) {
        discText->SetText("Server Disconnect");
    }
    startServerButton_->SetVisible(!serverConnection && !serverRunning);
    exitButton_->SetVisible(!serverConnection && !serverRunning);

    //  textEdit_->SetVisible(!serverConnection && !serverRunning);
}

void MayaScape::SetAerialCamera() {
    // Apply camera transformations
    cameraNode_->SetPosition(Vector3(0.0f, 180.0f, 0.0f));
    cameraNode_->SetRotation(Quaternion(90.0f, 0.0f, 0.0f));
}

void MayaScape::SetAerialCamera(const Vector3& target, float yaw) {

    Vector3 tgt;
    tgt = target;
    if (target.Equals(Vector3::ZERO)) {
        tgt = Vector3(0.0f, 90.0f, 0.0f);
    }
    // Apply camera transformations
    cameraNode_->SetPosition(Vector3(tgt.x_, tgt.y_+14.0f, tgt.z_));
    float delta = (yaw-360.0f)-cameraNode_->GetRotation().YawAngle();
//    URHO3D_LOGINFOF("--- yaw delta of cam vs. vehicle: %f", delta);
    cameraNode_->SetRotation(Quaternion(60.0f, cameraNode_->GetRotation().YawAngle()+(delta*0.98f), 0.0f));


//    URHO3D_LOGINFOF("--- yaw: %f", yaw);
}

void MayaScape::MoveCamera(Node *actorNode, float timeStep) {
    // Right mouse button controls mouse cursor visibility: hide when pressed
    UI *ui = GetSubsystem<UI>();
    Input *input = GetSubsystem<Input>();
    ui->GetCursor()->SetVisible(!input->GetMouseButtonDown(MOUSEB_RIGHT));

    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    if (started_) {

        // For clients
        if (!isServer_) {
            //URHO3D_LOGINFO("--- Moving camera ");

            // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch and only move the camera
            // when the cursor is hidden
            if (!ui->GetCursor()->IsVisible()) {
                IntVector2 mouseMove = input->GetMouseMove();
                yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
                pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
                pitch_ = Clamp(pitch_, 1.0f, 90.0f);
            }

            // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
           // cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

            // Only move the camera / show instructions if we have a controllable object
            bool showInstructions = false;
            if (playerObjectID_ != 0) {

                actorNode = scene_->GetNode(playerObjectID_);
                Node* vNode = scene_->GetNode(playerVehicleID_);
                if (vNode) {
                    RigidBody *rb = vNode->GetComponent<RigidBody>(true);

                    if (rb) {
                        URHO3D_LOGINFOF("--- Found rigid body: %u at (%f, %f, %f)", playerVehicleID_,
                                        rb->GetNode()->GetPosition().x_, rb->GetNode()->GetPosition().y_,
                                        rb->GetNode()->GetPosition().z_);
                    }
                }
                if (actorNode) {

                    const float CAMERA_DISTANCE = 8.0f;

                    Vector3 startPos = actorNode->GetPosition();
//                    URHO3D_LOGINFOF("--- Found controllable object -> position: %d [%f, %f, %f] ", playerObjectID_, actorNode->GetPosition().x_, actorNode->GetPosition().y_, actorNode->GetPosition().z_);

                    // Snap camera to vehicle once available

                    // smooth step
                    const float rotLerpRate = 10.0f;
                    const float maxVel = 50.0f;
                    const float damping = 0.2f;

                    Quaternion dir = Quaternion(0, 0, 90);

                    targetCameraPos_ = startPos + Vector3(0, 5.0f, CAMERA_DISTANCE);
                    // Calculate ray based on focus object
//                    float curDist = (focusObjects_[focusIndex_] - targetCameraPos_).Length();
                    float curDist = (actorNode->GetPosition() - targetCameraPos_).Length();
                    curDist = SpringDamping(curDist, CAMERA_DISTANCE, springVelocity_, damping, maxVel, timeStep);

                    // Calculate position based on focus object
                    Vector3 targetPos = actorNode->GetPosition() - dir * Vector3(0.0f, 0.0f, curDist);

                    // Set camera target position
                    targetCameraPos_ = targetPos;

                    Vector3 cameraTargetPos = targetCameraPos_;
                    Vector3 cameraStartPos = actorNode->GetPosition();

                    // Raycast camera against static objects (physics collision mask 2)
                    // and move it closer to the vehicle if something in between
                    Ray cameraRay(cameraStartPos, cameraTargetPos - cameraStartPos);
                    float cameraRayLength = (cameraTargetPos - cameraStartPos).Length();
                    PhysicsRaycastResult result;

                    if ((!isServer_) && (started_)) {
                        if (scene_->GetComponent<PhysicsWorld>()) {

                            scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, cameraRay, cameraRayLength,
                                                                                NETWORKACTOR_COL_LAYER);
                            if (result.body_)
                                cameraTargetPos = cameraStartPos + cameraRay.direction_ * (result.distance_ - 0.5f);
                        }
                    }

                    // Apply camera transformations
                    cameraNode_->SetPosition(cameraTargetPos);
                    //cameraNode_->SetRotation(dir);

                    // On client
                    if (!isServer_) {

                        if (started_) {

                            // Always show waypoints
//                            player_->DebugDraw(Color::MAGENTA);
                        }
                    }

                    if (drawDebug_) {
                        //          scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);
//            player_->GetVehicle()->DebugDraw(Color::MAGENTA);

//               terrain_->SetViewMask(0);

                        //            trackColShape_->DrawDebugGeometry(scene_->GetComponent<DebugRenderer>(), false);

                    }




/*
                    // Calculate ray based on focus object
                    float curDist = (focusObjects_[focusIndex_] - targetCameraPos_).Length();
                    curDist = SpringDamping(curDist, CAMERA_DISTANCE, springVelocity_, damping, maxVel, timeStep);

                    // Calculate position based on focus object
                    Vector3 targetPos = actorNode->GetPosition() - dir * Vector3(0.0f, 0.0f, curDist);

                    // Set camera target position
                    targetCameraPos_ = targetPos;

                    Vector3 cameraTargetPos = targetCameraPos_;
                    Vector3 cameraStartPos = actorNode->GetPosition();

                    // Raycast camera against static objects (physics collision mask 2)
                    // and move it closer to the vehicle if something in between
                    Ray cameraRay(cameraStartPos, cameraTargetPos - cameraStartPos);
                    float cameraRayLength = (cameraTargetPos - cameraStartPos).Length();
                    PhysicsRaycastResult result;

                    scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, cameraRay, cameraRayLength, NETWORKACTOR_COL_LAYER);
                    if (result.body_)
                        cameraTargetPos = cameraStartPos + cameraRay.direction_ * (result.distance_ - 0.5f);





                    Vector3 destPos =
                            actorNode->GetPosition() + cameraNode_->GetRotation() * Vector3::BACK * CAMERA_DISTANCE;
                    Vector3 seg = destPos - startPos;
                    Ray cameraRay(startPos, seg.Normalized());
                    float cameraRayLength = seg.Length();
                    PhysicsRaycastResult result;
                    scene_->GetComponent<PhysicsWorld>()->SphereCast(result, cameraRay, 1.0f, cameraRayLength,
                                                                     NETWORKACTOR_COL_LAYER);
                    if (result.body_)
                        destPos = startPos + cameraRay.direction_ * result.distance_;

                    // Move camera some distance away from the ball
                    cameraNode_->SetPosition(destPos);
*/



                    showInstructions = true;

//                    URHO3D_LOGINFO("--- Retrieved NetworkActor.");
                    SetAerialCamera(actorNode->GetPosition(), actorNode->GetRotation().YawAngle());

                    if (plyFltText_->GetNode()) {
                        plyFltText_->SetText(clientName_);
                        plyFltText_->GetNode()->SetPosition(actorNode->GetPosition() + Vector3(-13.0f, 10.0f, -5));
                    }



                } else {
                    URHO3D_LOGINFO("--- Could not get NetworkActor, aborting.");
                    //SaveScene(false);
                }
            } else {
                URHO3D_LOGINFO("--- Could not find controllable object, aborting.");
            }

        }
    }
}


void MayaScape::HandlePhysicsPreStep(StringHash eventType, VariantMap &eventData) {

//    URHO3D_LOGINFO("--- HandlePhysicsPreStep.");

    if (started_) {
        Server *server = GetSubsystem<Server>();
        Input *input = GetSubsystem<Input>();

        // set controls and pos
        ntwkControls_.yaw_ = yaw_;

        /*
     //   ntwkControls_.Set(NTWK_CTRL_FORWARD, input->GetKeyDown(KEY_W));
        ntwkControls_.Set(NTWK_CTRL_BACK, input->GetKeyDown(KEY_S));
        ntwkControls_.Set(NTWK_CTRL_LEFT, input->GetKeyDown(KEY_A));
        ntwkControls_.Set(NTWK_CTRL_RIGHT, input->GetKeyDown(KEY_D));
        ntwkControls_.Set(NTWK_SWAP_MAT, input->GetKeyDown(KEY_T));
*/
        char str[50];
//        sprintf(str, "[%f, %f, %f]", actorNode->GetPosition().x_, actorNode->GetPosition().y_, actorNode->GetPosition().z_);

        //        String hudText = "ActorNode Position: " + String(str);
        //      hudText_->SetText(hudText);
        //        sprintf(str, "[%f, %f, %f]", cameraNode_->GetRotation().YawAngle(), cameraNode_->GetRotation().RollAngle(), cameraNode_->GetRotation().PitchAngle());
//        sprintf(str, "[%f, %f, %s]", controls.pitch_, controls.yaw_,
//                ToStringHex(controls.buttons_).CString());


/*
        if (player_) {
            if (player_->vehicle_) {
                sprintf(str, "[%f, %f]", player_->vehicle_->lastAccel_, player_->vehicle_->lastSteer_);

                String hudText = "vehicle control: " + String(str);
                hudText_->SetText(hudText);

            }
        }
*/

        Node *actorNode = nullptr;
        String hudText = "";

        if (playerObjectID_ != 0) {
            actorNode = scene_->GetNode(playerObjectID_);

            if (actorNode) {
                // Controllable client network actor (replicated from server based on client controls)


                // axis
                const StringHash axisHashList[SDL_CONTROLLER_AXIS_MAX/2] = { VAR_AXIS_0, VAR_AXIS_1, VAR_AXIS_2 };
                // left stick - vehicle
                Variant lStick = ntwkControls_.extraData_[VAR_AXIS_0];
                Vector2 lAxisVal = lStick.GetVector2();

                // right stick
                Variant rStick = ntwkControls_.extraData_[VAR_AXIS_1];
                Vector2 rAxisVal = rStick.GetVector2();

                bool snap = false;
                ntwkControls_.Set(NTWK_CTRL_LEFT, 0);
                ntwkControls_.Set(NTWK_CTRL_RIGHT, 0);
                if (lAxisVal.x_ < -0.4f) {
                    // left
                    ntwkControls_.Set(NTWK_CTRL_LEFT, 1);
                    snap = true;
                } else if (lAxisVal.x_ > 0.4f) {
                    // right
                    ntwkControls_.Set(NTWK_CTRL_RIGHT, 1);
                    snap = true;
                }


                for (int i = 0; i < hudTextList_.Size(); i++) {
                    switch (i) {
                        case 0:
                            hudText = "Button state: " + String(ntwkControls_.buttons_);
                            break;
                        case 1:
                            hudText = "L axis: " + String(lAxisVal);
                            break;
                        case 2:
                            hudText = "R axis: " + String(rAxisVal);
                            break;
                        case 3:
                            hudText = "snap = " + String(snap);
                            break;
                    }

                    hudTextList_[i]->SetText(hudText);
                }

            }


        }


        server->UpdatePhysicsPreStep(ntwkControls_);

        if (isServer_) {
            using namespace Update;
            float timeStep = eventData[P_TIMESTEP].GetFloat();
            server->UpdateActors(timeStep);
        } else {
            // CLIENT UPDATE
            auto network = GetSubsystem<Network>();
            auto serverConnection = network->GetServerConnection();
            scene_ = serverConnection->GetScene();
          //  server->UpdateClient(serverConnection);
//            scene_ =
        }
        /*
        // This function is different on the client and server. The client collects controls (WASD controls + yaw angle)
        // and sets them to its server connection object, so that they will be sent to the server automatically at a
        // fixed rate, by default 30 FPS. The server will actually apply the controls (authoritative simulation.)
        auto network = GetSubsystem<Network>();
        auto serverConnection = network->GetServerConnection();

        // Client: collect controls
        if (serverConnection) {
            if (csp_client.prediction_controls != nullptr) {
                URHO3D_LOGDEBUG("PhysicsPreStep predict");

                if (clientObjectID_) {
                    auto playerNode = scene_->GetNode(clientObjectID_);
                    //if (playerNode != nullptr)
                       // apply_input(playerNode, *csp_client.prediction_controls);
                }
            } else {
                URHO3D_LOGDEBUG("PhysicsPreStep sample -> client sent controls");

                auto controls = sample_input();

                // predict locally
                if (clientObjectID_) {
                    auto playerNode = scene_->GetNode(clientObjectID_);
                  // if (playerNode != nullptr)
                        //apply_input(playerNode, controls);
                }

                // Set the controls using the CSP system
    //            csp_client.add_input(controls);
                serverConnection->SetControls(controls);

                // In case the server wants to do position-based interest management using the NetworkPriority components, we should also
                // tell it our observer (camera) position. In this sample it is not in use, but eg. the NinjaSnowWar game uses it
                serverConnection->SetPosition(cameraNode_->GetPosition());
            }
        }


            //Server: apply controls to client objects
        else if (network->IsServerRunning()) {




            URHO3D_LOGDEBUG("apply clients' controls");
            auto csp = scene_->GetComponent<CSP_Server>();

            const auto &connections = network->GetClientConnections();
            for (const auto &connection : connections) {
                if (csp->client_inputs[connection].empty())
                    continue;

                auto &controls = csp->client_inputs[connection].front();
                apply_input(connection, controls);
                csp->client_input_IDs[connection] = controls.extraData_["id"].GetUInt();
                csp->client_inputs[connection].pop();
            }
        }
    */
    }


}

void MayaScape::HandleConnectionFailed(StringHash eventType, VariantMap &eventData) {
    URHO3D_LOGINFO("Connection to server failed!");
    InitMsgWindow("Connection failure", "Connection to server failed!");
}

void MayaScape::HandleConnect(StringHash eventType, VariantMap &eventData) {
    static const int MAX_ARRAY_SIZE = 10;
    static String colorArray[MAX_ARRAY_SIZE] =
            {
                    "WHITE",
                    "GRAY",
                    "BLACK",
                    "RED",
                    "GREEN",
                    "BLUE",
                    "CYAN",
                    "MAGENTA",
                    "YELLOW",
                    "VEGAS GOLD"
            };

    // Client code
    isServer_ = false;
    Server *server = GetSubsystem<Server>();

    // Hard-coded game server address for now
//    String address = "www.monkeymaya.com";
//    String address = "localhost";
    String address = GAME_SERVER_ADDRESS;

//    String address = textEdit_->GetText().Trimmed();

    // randomize (or customize) client info/data
    int idx = Random(MAX_ARRAY_SIZE - 1);
    char buffer[200];
    String baseName = colorArray[idx];
    sprintf(buffer, "%s-%d", baseName.CString(), Random(1, 1000));
    String name = buffer;
    URHO3D_LOGINFOF("client idx=%i, username=%s", idx, name.CString());

    VariantMap identity;
    identity["UserName"] = name;
    identity["ColorIdx"] = idx;


    // Client connect to server
    if (server->Connect(address, SERVER_PORT, identity)) {

        URHO3D_LOGINFOF("client identity name=%s", name.CString());
        URHO3D_LOGINFOF("HandleClientConnected - data: [%s, %d]", name.CString(), idx);

        // Store in local login list
        loginList_.Push(name.CString());

        // Change UI -> hide menu and show game
        UpdateButtons();
        // Switch to game mode
        UpdateUIState(true);
        started_ = true;
        // Set logo sprite alignment
        logoSprite_->SetAlignment(HA_CENTER, VA_BOTTOM);
        logoSprite_->SetPosition(-280, -3);

        // Make logo not fully opaque to show the scene underneath
        logoSprite_->SetOpacity(0.3f);

        // Client startup code

        // Store name
        clientName_ = name.CString();

        String playerText = "Logged in as: " + String(clientName_.CString());
        instructionsText_->SetText(playerText);
        instructionsText_->SetPosition(0, 730);



        String address = textEdit_->GetText().Trimmed();
        // Empty the text edit after reading the address to connect to
        textEdit_->SetText(String::EMPTY);

        UpdateButtons();


        //URHO3D_LOGINFOF("Client: Scene checksum -> %d", scene_->GetChecksum());

        // Save initial scene for debugging
        //SaveScene(true);



        // On client
        //player_->vehicle_->SetVisible(true);


        //     player_->vehicle_->SetVisible(true);
        /*
                // Create a directional light with shadows
                Node* lightNode = scene_->CreateChild("DirectionalLight", LOCAL);
                lightNode->SetDirection(Vector3(0.3f, -0.5f, 0.425f));
                Light* light = lightNode->CreateComponent<Light>();
                light->SetLightType(LIGHT_DIRECTIONAL);
                light->SetCastShadows(true);
                light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
                light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
                light->SetSpecularIntensity(0.5f);
        */
        URHO3D_LOGINFOF("client idx=%i, username=%s", idx, name.CString());
        // Clear client replicated objects
        scene_->MarkNetworkUpdate();

    } else {
        URHO3D_LOGINFOF("Connection to server failed =%s", address.CString());
        engine_->Exit();
    }
}

void MayaScape::HandleDisconnect(StringHash eventType, VariantMap &eventData) {
    Server *server = GetSubsystem<Server>();
    server->Disconnect();

    UpdateButtons();
}

void MayaScape::HandleStartServer(StringHash eventType, VariantMap &eventData) {
    Server *server = GetSubsystem<Server>();
    if (!server->StartServer(SERVER_PORT)) {
        engine_->Exit();
    }

    // Save initial scene for server
    // initialScene_ = SaveScene(true);

    // Server code
    //File sceneFile(context_, initialScene_, FILE_READ);
    //server->InitializeScene(sceneFile);

    // create Admin
    CreateAdminPlayer();

    UpdateButtons();
    // Switch to game mode
    UpdateUIState(true);
    started_ = true;

    // Disable updates (allow focus on processing for server)
//    scene_->SetUpdateEnabled(false);

    // Set logo sprite alignment
    logoSprite_->SetAlignment(HA_CENTER, VA_BOTTOM);
    logoSprite_->SetPosition(-280, -3);

    // Make logo not fully opaque to show the scene underneath
    logoSprite_->SetOpacity(0.3f);

    Renderer *renderer = GetSubsystem<Renderer>();
    Network *network = GetSubsystem<Network>();
    Connection *serverConnection = network->GetServerConnection();
    bool serverRunning = network->IsServerRunning();
//    renderer->Set
    //renderer->SetViewport(0, viewport);

}

void MayaScape::HandleConnectionStatus(StringHash eventType, VariantMap &eventData) {
    using namespace ServerStatus;
    StringHash msg = eventData[P_STATUS].GetStringHash();

    if (msg == E_SERVERDISCONNECTED) {
        scene_->RemoveAllChildren();
        CreateScene();
        SetupViewport();

        URHO3D_LOGINFO("server connection lost");
    }

    UpdateButtons();

}

void MayaScape::HandleClientObjectID(StringHash eventType, VariantMap &eventData) {
    // On client
    // server requires client hash and scene info
    Server *server = GetSubsystem<Server>();

    URHO3D_LOGINFO("*** HandleClientObjectID");


    // Client stores client object id
    loginClientObjectID_ = eventData[ClientObjectID::P_ID].GetUInt();

    URHO3D_LOGINFOF("Client -> HandleClientObjectID: %u", loginClientObjectID_);

    URHO3D_LOGINFOF("Client -> scene checksum: %d", ToStringHex(scene_->GetChecksum()).CString());

//    scene_->MarkNetworkUpdate();

    auto *network = GetSubsystem<Network>();
    Connection *serverConnection = network->GetServerConnection();

    if (serverConnection) {

        scene_ = serverConnection->GetScene();

        SaveScene(true);
        /*
        // A VectorBuffer object is convenient for constructing a message to send
        VectorBuffer msg;
        msg.WriteString("hello!");
        // Send the chat message as in-order and reliable
        serverConnection->SendMessage(MSG_NODE_ERROR, true, true, msg);
        // Empty the text edit after sending
        textEdit_->SetText(String::EMPTY);
*/
   //     URHO3D_LOGINFOF("Client -> sent node error message: %s", msg.GetData());
    }
}


void MayaScape::HandleExit(StringHash eventType, VariantMap &eventData) {
    engine_->Exit();
}

void MayaScape::InitMsgWindow(String title, String message) {
    UI *ui = GetSubsystem<UI>();
    ResourceCache *cache = GetSubsystem<ResourceCache>();

    // Create the Window and add it to the UI's root node
    msgWindow_ = new Window(context_);
    ui->GetRoot()->AddChild(msgWindow_);

    // Set Window size and layout settings
    msgWindow_->SetMinWidth(384);
    msgWindow_->SetLayout(LM_VERTICAL, 6, IntRect(6, 6, 6, 6));
    msgWindow_->SetAlignment(HA_CENTER, VA_CENTER);
    msgWindow_->SetName(message);


    // Create Window 'titlebar' container
    auto *titleBar = new UIElement(context_);
    titleBar->SetMinSize(0, 24);
    titleBar->SetVerticalAlignment(VA_TOP);
    titleBar->SetLayoutMode(LM_HORIZONTAL);

    // Create the Window title Text
    auto *windowTitle = new Text(context_);
    windowTitle->SetName("WindowTitle");
    windowTitle->SetText(title);
//    windowTitle->SetFont(cache->GetResource<Font>("Fonts/SinsGold.ttf"), 15);

    // Create the Window's close button
    auto *buttonClose = new Button(context_);
    buttonClose->SetName("CloseButton");

    // Add the controls to the title bar
    titleBar->AddChild(windowTitle);
    titleBar->AddChild(buttonClose);

    // Add the title bar to the Window
    msgWindow_->AddChild(titleBar);

    // Apply styles
    msgWindow_->SetStyleAuto();
    windowTitle->SetStyleAuto();
    buttonClose->SetStyle("CloseButton");

    // Subscribe to buttonClose release (following a 'press') events
    SubscribeToEvent(buttonClose, E_RELEASED, URHO3D_HANDLER(MayaScape, HandleClosePressed));

    // Subscribe also to all UI mouse clicks just to see where we have clicked
    //  SubscribeToEvent(E_UIMOUSECLICK, URHO3D_HANDLER(HelloGUI, HandleControlClicked));
}

void MayaScape::HandleClosePressed(StringHash eventType, VariantMap &eventData) {
    // Shutdown
    engine_->Exit();
}


void MayaScape::OutputLoginListToConsole() {

    URHO3D_LOGINFO("**** CLIENT: CLIENT LIST *****************************************************");

    for (int i = 0; i < loginList_.Size(); i++) {
        URHO3D_LOGINFOF("**** LOGIN: %s", loginList_.At(i).CString());
    }

    URHO3D_LOGINFO("******************************************************************************");

}


void MayaScape::ShowChatText(const String &row) {
    chatHistory_.Erase(0);
    chatHistory_.Push(row);

    // Concatenate all the rows in history
    String allRows;
    for (unsigned i = 0; i < chatHistory_.Size(); ++i)
        allRows += chatHistory_[i] + "\n";

    chatHistoryText_->SetText(allRows);
}

void MayaScape::HandleLogMessage(StringHash /*eventType*/, VariantMap &eventData) {
    using namespace LogMessage;

    ShowChatText(eventData[P_MESSAGE].GetString());
}

void MayaScape::HandleSend(StringHash /*eventType*/, VariantMap &eventData) {
    String text = textEdit_->GetText();
    if (text.Empty())
        return; // Do not send an empty message

    auto *network = GetSubsystem<Network>();
    Connection *serverConnection = network->GetServerConnection();

    if (serverConnection) {
        // A VectorBuffer object is convenient for constructing a message to send
        VectorBuffer msg;
        msg.WriteString(text);
        // Send the chat message as in-order and reliable
        serverConnection->SendMessage(MSG_CHAT, true, true, msg);
        // Empty the text edit after sending
        textEdit_->SetText(String::EMPTY);
    }
}

void MayaScape::HandleNetworkMessage(StringHash /*eventType*/, VariantMap &eventData) {
    auto *network = GetSubsystem<Network>();

    using namespace NetworkMessage;

    int msgID = eventData[P_MESSAGEID].GetInt();

    URHO3D_LOGINFOF("HandleNetworkMessage: msgID -> %d", msgID);

    if (msgID == MSG_NODE_ERROR) {
        // Client cannot get Network Actor, resend
        //    scene_->MarkReplicationDirty(scene_);
        ///    scene_->MarkNetworkUpdate();
    }

    scene_->MarkNetworkUpdate();


    if (msgID == MSG_CHAT) {
        const PODVector<unsigned char> &data = eventData[P_DATA].GetBuffer();
        // Use a MemoryBuffer to read the message data so that there is no unnecessary copying
        MemoryBuffer msg(data);
        String text = msg.ReadString();

        // If we are the server, prepend the sender's IP address and port and echo to everyone
        // If we are a client, just display the message
        if (network->IsServerRunning()) {
            auto *sender = static_cast<Connection *>(eventData[P_CONNECTION].GetPtr());

            text = sender->ToString() + " " + text;

            VectorBuffer sendMsg;
            sendMsg.WriteString(text);
            // Broadcast as in-order and reliable
            network->BroadcastMessage(MSG_CHAT, true, true, sendMsg);
        }

        ShowChatText(text);
    }
}

String MayaScape::SaveScene(bool initial) {
    String filename = "MayaScape_demo";
    if (!initial)
        filename += "InGame";
    File saveFile(context_, GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Scenes/" + filename + ".xml",
                  FILE_WRITE);
    scene_->SaveXML(saveFile);
    return saveFile.GetName();
}


Controls MayaScape::sample_input() {
    auto ui = GetSubsystem<UI>();
    auto input = GetSubsystem<Input>();

    Controls controls;

    // Copy mouse yaw
    controls.yaw_ = yaw_;

    // Only apply WASD controls if there is no focused UI element
    if (!ui->GetFocusElement()) {
        controls.Set(CTRL_FORWARD, input->GetKeyDown(KEY_W));
        controls.Set(CTRL_BACK, input->GetKeyDown(KEY_S));
        controls.Set(CTRL_LEFT, input->GetKeyDown(KEY_A));
        controls.Set(CTRL_RIGHT, input->GetKeyDown(KEY_D));
    }

    return controls;
}


void MayaScape::apply_input(Node *actorNode, const Controls &controls) {
    // Torque is relative to the forward vector
    Quaternion rotation(0.0f, controls.yaw_, 0.0f);

#define CSP_TEST_USE_PHYSICS // used for testing to make sure problems aren't related to the physics
#ifdef CSP_TEST_USE_PHYSICS
    auto *body = actorNode->GetComponent<RigidBody>(true);
    const float MOVE_TORQUE = 3.0f;

    if (body) {

        auto change_func = [&](Vector3 force) {
            //#define CSP_TEST_USE_VELOCITY
#ifdef CSP_TEST_USE_VELOCITY
            body->ApplyForce(force);
#else
            body->ApplyTorque(force);
#endif
        };

        // Movement torque is applied before each simulation step, which happen at 60 FPS. This makes the simulation
        // independent from rendering framerate. We could also apply forces (which would enable in-air control),
        // but want to emphasize that it's a ball which should only control its motion by rolling along the ground
        if (controls.buttons_ & CTRL_FORWARD)
            change_func(rotation * Vector3::RIGHT * MOVE_TORQUE);
        if (controls.buttons_ & CTRL_BACK)
            change_func(rotation * Vector3::LEFT * MOVE_TORQUE);
        if (controls.buttons_ & CTRL_LEFT)
            change_func(rotation * Vector3::FORWARD * MOVE_TORQUE);
        if (controls.buttons_ & CTRL_RIGHT)
            change_func(rotation * Vector3::BACK * MOVE_TORQUE);
#else
        const float move_distance = 2.f / scene->GetComponent<PhysicsWorld>()->GetFps();

        // Movement torque is applied before each simulation step, which happen at 60 FPS. This makes the simulation
        // independent from rendering framerate. We could also apply forces (which would enable in-air control),
        // but want to emphasize that it's a ball which should only control its motion by rolling along the ground
        if (controls.buttons_ & CTRL_FORWARD)
            ballNode->SetPosition(ballNode->GetPosition() + Vector3::RIGHT * move_distance);
        if (controls.buttons_ & CTRL_BACK)
            ballNode->SetPosition(ballNode->GetPosition() + Vector3::LEFT * move_distance);
        if (controls.buttons_ & CTRL_LEFT)
            ballNode->SetPosition(ballNode->GetPosition() + Vector3::FORWARD * move_distance);
        if (controls.buttons_ & CTRL_RIGHT)
            ballNode->SetPosition(ballNode->GetPosition() + Vector3::BACK * move_distance);
#endif

    }
}

void MayaScape::apply_input(Connection *connection, const Controls &controls) {
    auto ballNode = serverObjects_[connection];
    if (!ballNode)
        return;

    apply_input(ballNode, controls);
}


void MayaScape::HandleSceneUpdate(StringHash eventType, VariantMap &eventData) {
    // Move the camera by touch, if the camera node is initialized by descendant sample class
    if (cameraNode_) {
        auto input = GetSubsystem<Input>();
        for (unsigned i = 0; i < input->GetNumTouches(); ++i) {
            auto state = input->GetTouch(i);
            if (!state->touchedElement_)    // Touch on empty space
            {
                if (state->delta_.x_ || state->delta_.y_) {
                    auto camera = cameraNode_->GetComponent<Camera>();
                    if (!camera)
                        return;

                    auto graphics = GetSubsystem<Graphics>();
                    yaw_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.x_;
                    pitch_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.y_;

                    // Construct new orientation for the camera scene node from yaw and pitch; roll is fixed to zero
                    cameraNode_->SetRotation({pitch_, yaw_, 0.0f});
                } else {
                    // Move the mouse to the touch position
                    if (input->IsMouseVisible())
                        input->SetMousePosition(state->position_);
                }
            }
        }
    }

}

