//
// Copyright (c) 2008-2016 the Urho3D project.
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
#include <Urho3D/Core/Context.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/UI/Text3D.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Network/Connection.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Math/MathDefs.h>
#include <Urho3D/Graphics/DebugRenderer.h>

#include "NetworkActor.h"

#include <Urho3D/DebugNew.h>
#include <MayaScape/Missile.h>

#define PI 3.1415926

//=============================================================================
//=============================================================================
NetworkActor::NetworkActor(Context *context)
        : ClientObj(context), mass_(10.0f) {
    //  SetUpdateEventMask(0);
    // fixed update() for inputs and post update() to sync wheels for rendering
//    SetUpdateEventMask( USE_FIXEDUPDATE | USE_FIXEDPOSTUPDATE| USE_POSTUPDATE );

    ///Set original staus
    SetSpeed(0.0f);
    SetMaxSpeed(5.0f);
    SetDamping(0.015f);
    SetAcceleration(0.03f);
    SetBrake(0.05f);
    SetTowards(Vector3(0.0f, 1.0f, 0.0f));
    SetTurningVelocity(100.0f);
    SetBulletType("AP");
    ///Set bullets type
//    bulletType_ = "CB";
    lastFire_ = 0;
    targetAgentIndex_ = 0;
//    bulletType_ = "AP";

}

NetworkActor::~NetworkActor() {

    URHO3D_LOGINFOF("**** DESTROYING NetworkActor OBJECT -> %d", this->id_);

    if (vehicle_) {
        URHO3D_LOGINFOF("**** DESTROYING CLIENT VEHICLE OBJECT -> %d", this->id_);
//        vehicle_->GetNode()->RemoveAllChildren();
        vehicle_->Remove();
    }

    if (node_) {
        URHO3D_LOGINFOF("**** DESTROYING CLIENT NODE OBJECT -> %d", this->id_);
        node_->Remove();
    }

}

void NetworkActor::RegisterObject(Context *context) {
    context->RegisterFactory<NetworkActor>();

    URHO3D_COPY_BASE_ATTRIBUTES(ClientObj);

    // Network Actor Network Attributes
 //   URHO3D_ATTRIBUTE("Name", String, userName_, String::EMPTY, AM_DEFAULT | AM_NET);
 //   URHO3D_ATTRIBUTE("Color Index", int, colorIdx_, 0, AM_DEFAULT | AM_NET);

    // These macros register the class attributes to the Context for automatic load / save handling.
    // We specify the Default attribute mode which means it will be used both for saving into file, and network replication
    //URHO3D_ATTRIBUTE("Controls Yaw" int, controls_.buttons_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Speed", float, speed_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Turning Velocity", float, turningVelocity_, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Is Enabled", IsEnabled, SetEnabled, bool, true, AM_DEFAULT);

    //    URHO3D_ATTRIBUTE("On Ground", bool, vehicle_->GetRaycastVehicle()->GetCurrentSpeedKmHour(), 0.0f, AM_DEFAULT);

    //    URHO3D_ATTRIBUTE("On Ground", bool, onGround_, false, AM_DEFAULT);
//    URHO3D_ATTRIBUTE("OK To Jump", bool, okToJump_, true, AM_DEFAULT);
//    URHO3D_ATTRIBUTE("In Air Timer", float, inAirTimer_, 0.0f, AM_DEFAULT);
/*
    URHO3D_ATTRIBUTE("Player Name", String, name_, 0, AM_DEFAULT | AM_NET);
    URHO3D_ATTRIBUTE("Player Node", Variant, nodeInfo_->GetAttributeDefault(0), 0, AM_DEFAULT | AM_NET | AM_LATESTDATA);
    //URHO3D_ATTRIBUTE("Vehicle", Variant, vehicle_, 0, AM_DEFAULT | AM_NET | AM_LATESTDATA);*/
//    AM_LATESTDATA
    // update serializable of the change
//    SetAttribute("Player Node", Variant(idx));
    // update serializable of the change
 //   SetAttribute("Vehicle", Variant(vehicle_));
    /*
    String name_;
    WeakPtr<Node> nodeInfo_;
    Controls prevControls_;

    /// The controllable vehicle component.
    SharedPtr<Vehicle> vehicle_;
    SharedPtr<Text3D> floatingText_;

    bool created_;

    /// Flag when player is dead.
    bool killed_;
    bool isAI_;
    int agentIndex;
    int id_;
    int type_;
    unsigned int wpActiveIndex_;
    int targetAgentIndex_;


    /// Bullets
    String bulletType_;
    Vector<Vector3>* waypoints_ = nullptr;
    Vector3 toTarget_ = Vector3::ZERO;


    //    btCollisionShape                    *sphShape_;

    /// parameter

    float mass_;
    float speed_;
    float maxSpeed_;
    float damping_;
    float acceleration_;
    float brake_;
    Vector3 towards_;
    Vector2 towards2d_;
    float turningVelocity_;
    float lastFire_;
    float heading_;
    bool isReady_;
    int life_;
    int score_;
    int health_;

    Vector3 force_;
    Vector3 offset_;

    float changeTargetTime_;
    bool autoSteering_;

    bool doJump_;
    */

}

void NetworkActor::ApplyAttributes() {
}

void NetworkActor::SetScene(Scene* scene)
{
    scene_ = scene;
}

void NetworkActor::Create(Connection* connection)
{
    connection_ = connection;

    auto* cache = GetSubsystem<ResourceCache>();

    // Create the scene node & visual representation. This will be a replicated object
    node_ = scene_->CreateChild("NetworkActor", REPLICATED);
    node_->AddTag("Player");
//    node_->SetVar("GUID", connection->GetGUID());
    node_->SetPosition(Vector3(0, 10, 0));
    node_->SetScale(0.5f);
    auto* ballObject = node_->CreateComponent<StaticModel>();
    ballObject->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
    ballObject->SetMaterial(cache->GetResource<Material>("Materials/StoneSmall.xml"));

    floatingText_ = node_->CreateComponent<Text3D>(REPLICATED);
//    titleText->SetText(connection->GetGUID());
    floatingText_->SetFaceCameraMode(FaceCameraMode::FC_LOOKAT_XYZ);
    floatingText_->SetFont(cache->GetResource<Font>("Fonts/BlueHighway.sdf"), 30);

    // Create the physics components
    auto* body = node_->CreateComponent<RigidBody>();
    body->SetMass(1.0f);
    body->SetFriction(1.0f);
    // In addition to friction, use motion damping so that the ball can not accelerate limitlessly
    body->SetLinearDamping(0.5f);
    body->SetAngularDamping(0.5f);
    //body->SetLinearVelocity(Vector3(0.1, 1, 0.1));
    auto* shape = node_->CreateComponent<CollisionShape>();
    shape->SetSphere(1.0f);

    // Create a random colored point light at the ball so that can see better where is going
    auto* light = node_->CreateComponent<Light>();
    light->SetRange(3.0f);
    light->SetColor(Color(0.5f + ((unsigned)Rand() & 1u) * 0.5f, 0.5f + ((unsigned)Rand() & 1u) * 0.5f, 0.5f + ((unsigned)Rand() & 1u) * 0.5f));

    //node_->SetScale(1.0f);

    // Init vehicle
    Node *vehicleNode = scene_->CreateChild("Vehicle", REPLICATED);
    // Default at (0,300,0) above terrain before we set location
    float factor = 500.0f;

        // Place on track
//           vehicleNode->SetPosition(Vector3(-814.0f+Random(-400.f, 400.0f), 500.0f, -595.0f+Random(-400.f, 400.0f)));
        vehicleNode->SetPosition(Vector3(-814.0f+Random(-400.f, 400.0f), 300.0f, -595.0f+Random(-400.f, 400.0f)));

        // Create the vehicle logic component
        vehicle_ = vehicleNode->CreateComponent<Vehicle>(REPLICATED);
        vehicle_->Init(true);
        vehicle_->Create();
//        GetNode()->SetPosition(vehicle_->GetNode()->GetPosition());

        wpActiveIndex_ = 0;
        targetAgentIndex_ = 0;

        vehicleNode->SetRotation(Quaternion(0.0, -90.0, 0.0));


        // register
        SetUpdateEventMask(USE_FIXEDUPDATE);
}

void NetworkActor::SetNode(Node* node)
{
    node_ = node;
}

void NetworkActor::SetConnection(Connection* connection)
{
    connection_ = connection;
}

void NetworkActor::SwapMat() {
    ResourceCache *cache = GetSubsystem<ResourceCache>();

    int idx = Random(MAX_MAT_COUNT);
    while (idx == colorIdx_) {
        idx = Random(MAX_MAT_COUNT);
    }

    // update serializable of the change
    SetAttribute("Color Index", Variant(idx));

    String matName = ToString("NetDemo/ballmat%i.xml", colorIdx_);
    StaticModel *ballModel = node_->GetComponent<StaticModel>();
    ballModel->SetMaterial(cache->GetResource<Material>(matName));
}

void NetworkActor::SetControls(Controls controls) {
    controls_ = controls;

    if (vehicle_) {
        // Apply control to vehicle
        vehicle_->controls_ = controls;
//    URHO3D_LOGDEBUG("NetworkActor::SetControls -> applying to physics world.");
    }
}

void NetworkActor::FixedUpdate(float timeStep) {
    if (!node_) {
        return;
    }

    // update prev
    lastFire_ += timeStep;

    // Client will only do local scene updates

    // Only allow server to control objects based on received controls from clients
        // SERVER CODE

        // Snap network actor position/rotation to vehicle
        if (vehicle_) {
            node_->SetPosition(vehicle_->GetNode()->GetPosition());
            node_->SetRotation(vehicle_->GetNode()->GetRotation());
        }

        /// Clients should not update the component on its own (server will handle it)

        // Read control data and apply to vehicle controller
        ///Acceleration
        if (controls_.IsDown(NTWK_CTRL_FORWARD)) {
            Accelerate();
            URHO3D_LOGINFOF("NetworkActor: FixedUpdate - applying ACCELERATE = %f", speed_);
        }
            ///Damping
        else {
            Damping();
        }
        ///Braking
        if (controls_.IsDown(NTWK_CTRL_BACK)) {
            Brake();
        }
        ///Turn left
        if (controls_.IsDown(NTWK_CTRL_LEFT)) {
            //Turn left
            //towards_ = Vector3(towards_.x_*cos(turningVelocity_*timeStep) - towards_.y_*sin(turningVelocity_*timeStep), towards_.x_*sin(turningVelocity_*timeStep) + towards_.y_*cos(turningVelocity_*timeStep), 0.0f);
            node_->Rotate2D(turningVelocity_ * timeStep);
            // The angle between rotation2d and x-axis
            float angle = 90.0f + node_->GetRotation2D();
            // The towards vector according to the angle
            towards_ = Vector3(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f), 0.0f);
        }
        ///Turn right
        if (controls_.IsDown(NTWK_CTRL_RIGHT)) {
            //Turn right
            //towards_ = Vector3(towards_.x_*cos(turningVelocity_*timeStep) + towards_.y_*sin(turningVelocity_*timeStep), -towards_.x_*sin(turningVelocity_*timeStep) + towards_.y_*cos(turningVelocity_*timeStep), 0.0f);
            node_->Rotate2D(-turningVelocity_ * timeStep);
            // The angle between rotation2d and x-axis
            float angle = 90.0f + node_->GetRotation2D();
            // The towards vector according to the angle
            towards_ = Vector3(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f), 0.0f);
        }



/* AUTO-STEERING CODE
    if (toTarget_ != Vector3::ZERO) {
        // Only pass once rigid body is setup
        if (pRigidBody_) {
            // Compute steer force
//            ComputeSteerForce();
            if (force_ != Vector3::ZERO) {

//            force_ = Vector3(1.0f, 0.0f, 1.0f);

                if (wpActiveIndex_ < 0)
                    return;

//                float wpOffsetX = -mapDim_ / 2;
//                float wpOffsetY = -mapDim_ / 2;
                // Convert marker position to world position for track
                //          float wpPosX = (((float)waypoints_->At(wpActiveIndex_).x_ / (float)miniMapWidth_)*mapDim_)+wpOffsetX;
//            float wpPosZ = (((float)waypoints_->At(wpActiveIndex_).z_ / (float)miniMapHeight_)*mapDim_)+wpOffsetY;


                //Vector3 tgt = Vector3(waypoints_->At(wpActiveIndex_).x_, waypoints_->At(wpActiveIndex_).y_, waypoints_->At(wpActiveIndex_).z_);


                // Calculate distance to waypoint
                Vector3 v = vehicle_->GetNode()->GetPosition() - toTarget_;// + Vector3(-1500,0,-1500);
                float steering = v.Normalized().DotProduct((vehicle_->GetNode()->GetDirection()))+0.4f;
/*
                if (steering > 1.0f) {
                    steering = -1.0f;
                }

                if (steering < -1.0f) {
                    steering = 1.0f;
                }
*/


        /*    if (autoSteering_) {
                URHO3D_LOGINFOF("***** Player AUTO-STEERING ENABLED - Vehicle Steer [%f]", vehicle_->GetSteering());
                URHO3D_LOGINFOF("***** Player - waypoint [%d]=[%f,%f,%f,%f]", wpActiveIndex_,
                                waypoints_->At(wpActiveIndex_).x_, waypoints_->At(wpActiveIndex_).y_,
                                waypoints_->At(wpActiveIndex_).z_, steering);
                URHO3D_LOGINFOF("***** Player - target =[%f,%f,%f]", toTarget_.x_, toTarget_.y_, toTarget_.z_);

                // Enable auto-steering
                vehicle_->UpdateSteering(steering);
            }*/
        //vehicle_->GetRigidBody()->
//            pRigidBody_->ApplyForce(force_);
/*

            Vector3 vel = pRigidBody_->GetLinearVelocity();

            float d = vel.Length();
            if (d < 10.0f) {
                d = 10.0f;
                pRigidBody_->SetLinearVelocity(vel.Normalized() * d);
            } else if (d > 50.0f) {
                d = 50.0f;
                pRigidBody_->SetLinearVelocity(vel.Normalized() * d);
            }

            Quaternion endRot = Quaternion(0, 0, 0);
            Vector3 nVel = vel.Normalized();
            endRot.FromLookRotation(nVel, Vector3::UP);
            endRot = endRot * Quaternion(90, 0, 0);
            pRigidBody_->SetRotation(endRot);

            Vector3 p = pRigidBody_->GetPosition();
            if (p.y_ < 10.0f) {
                p.y_ = 10.0f;
                pRigidBody_->SetPosition(p);
            } else if (p.y_ > 150.0f) {
                p.y_ = 150.0f;
                pRigidBody_->SetPosition(p);
            }
            }


        }
    }

*/
        ////

}


void NetworkActor::ComputeSteerForce() {

    //set the force member variable to zero
    force_ = Vector3(0, 0, 0);

    if (!waypoints_)
        return;

    if ((waypoints_->Empty())) {
        return;
    }
    //Attraction force
/*
    Vector3 CoM; //centre of mass, accumulated total
    int nAttract = 0; //count number of neighbours
    //set the force member variable to zero
    force_ = Vector3(0, 0, 0);
    //Search Neighbourhood
    for (int i = 0; i < numberOfBoids; i++)
    {
        //the current boid?
        if (this == &boidList[i]) continue;
        //sep = vector position of this boid from current oid
        Vector3 sep = pRigidBody->GetPosition() - boidList[i].pRigidBody->GetPosition();
        float d = sep.Length(); //distance of boid
        if (d < Range_FAttract)
        {
            //with range, so is a neighbour
            CoM += boidList[i].pRigidBody->GetPosition();
            nAttract++;
        }
    }
    if (nAttract > 0)
    {
        CoM /= nAttract;
        Vector3 dir = (CoM - pRigidBody->GetPosition()).Normalized();
        Vector3 vDesired = dir * FAttract_Vmax;
        force += (vDesired - pRigidBody->GetLinearVelocity())*FAttract_Factor;
    }
    if (nAttract > 5)
    {
        // stop checking once 5 neighbours have been found
        return;
    }

    //seperation force
    Vector3 sepForce;
    int nRepel = 0;
    for (int i = 0; i < numberOfBoids; i++)
    {
        //the current boid?
        if (this == &boidList[i]) continue;
        //sep = vector position of this boid from current oid
        Vector3 sep = pRigidBody->GetPosition() - boidList[i].pRigidBody->GetPosition();
        float d = sep.Length(); //distance of boid
        if (d < Range_FRepel)
        {
            sepForce += (sep / sep.Length());
            nRepel++;
        }
    }
    if (nRepel > 0)
    {
        sepForce *= FRepel_Factor;
        force += sepForce;
    }
    if (nRepel > 5)
    {
        // stop checking once 5 neighbours have been found
        return;
    }
    */

    //Allignment direction
    Vector3 align;
    int nAlign = 0;


    //sep = vector position of this boid from current oid

//    Vector3 sep = pRigidBody->GetPosition() - boidList[i].pRigidBody->GetPosition();
//    float d = sep.Length(); //distance of boid
//    if (d < Range_FRepel)


    Vector3 toTarget;
    if (!waypoints_->Empty()) {

//        URHO3D_LOGDEBUGF("Player::ComputeSteerForce() waypoints -> [%d] -> set to  %d", waypoints_->Size(), wpActiveIndex_);

        // Calculate distance to waypoint
        toTarget = vehicle_->GetNode()->GetPosition() - waypoints_->At(wpActiveIndex_);
    } else {
        return;
    }


    //    Vector3 dir = sep.Normalized();
//    float d = sep.Length(); // distance of boid

    //float steer = toTarget * speed / d;

    float speed = 1.0f;
    Vector3 desiredVelocity = toTarget.Normalized() * speed;


    /*
    if (d < Range_FAlign)
    {
        align += boidList[i].pRigidBody->GetLinearVelocity();
        nAlign++;
    }*/

    force_ += ToVector3(ToBtVector3(desiredVelocity) - vehicle_->GetRaycastVehicle()->GetBody()->getLinearVelocity());

    /*
    if (nAlign > 0)
    {
        align /= nAlign;

        Vector3 finalVel = align;

//        force_ += (finalVel - pRigidBody_->GetLinearVelocity()) * FAlign_Factor;
    }
    if (nAlign > 5)
    {
        // stop checking once 5 neighbours have been found
        return;
    }*/
}


void NetworkActor::Fire() {
    Fire(toTarget_);
}

void NetworkActor::Fire(Vector3 target) {
    Scene *scene = GetScene();


    URHO3D_LOGDEBUG("NetworkActor::Fire()");

    // 4test
    // Only for test
    if (bulletType_ == "AP") {
/*
		testcnt_++;
		Node* bullet0 = scene->CreateChild("bullet", REPLICATED);
		bullet0->CreateComponent<AP>(LOCAL);
		// Set the ownership of the bullet to the Player
	//	bullet0->GetComponent<AP>()->SetProducer(node);
		// Set the position and rotation of the bullet
		bullet0->SetWorldPosition(node->GetPosition() + towards_.Normalized()*0.2f);
		bullet0->SetWorldRotation(Quaternion(Vector3::UP, towards_));
//		bullet0->GetComponent<RigidBody2D>()->SetLinearVelocity(Vector2(towards_.x_, towards_.y_).Normalized() * 10.0f);
        URHO3D_LOGDEBUGF("Player::Fire() -> %d", testcnt_);*/
    } else {
        //
        // bulletType_ = "CB"

        /*
        if (testcnt_ > 4) {
            return;
        }*/

/*
        VariantMap& eventData = GetNode()->GetEventDataMap();
        eventData[P_DATA] = GetNode()->GetWorldPosition();
        SendEvent(StringHash("Blast"), eventData);

        GetNode()->Remove();
*/

        SharedPtr<Node> n;
        Node *bullet0 = scene->CreateChild("bullet", REPLICATED);
        Missile *newM = bullet0->CreateComponent<Missile>(LOCAL);
        newM->SetProducer(vehicle_->GetNode()->GetID());


        // Store local missile list
        //missileList_.Push(vehicle_->GetNode()->GetID());

//        VariantMap& eventData = GetNode()->GetEventDataMap();
//        eventData["owner"] = SharedPtr<Player>(this);

//        eventData["missileOwner"] = this->GetID();
        //       vehicle_->SendEvent(E_NODECOLLISION, eventData);



        // Set the ownership of the bullet to the Player
//        bullet0->GetComponent<Missile>()->SetProducer(SharedPtr<Node>(vehicle_->GetNode()));

        Node *tgt = scene->CreateChild("missileTarget", LOCAL);
        //tgt->>SetPosition(0f,0f,0f);
        tgt->SetPosition(target);
        newM->AddTarget(SharedPtr<Node>(tgt));
        // Assign the producer node
        newM->AssignProducer(vehicle_->GetNode()->GetID(),
                             vehicle_->GetNode()->GetPosition() + Vector3(0.0f, 2.0f, 0.0f));
        URHO3D_LOGDEBUGF("NetworkActor::Fire() [%d] -> [%f,%f,%f]", vehicle_->GetNode()->GetID(),
                         newM->GetNode()->GetPosition().x_,
                         newM->GetNode()->GetPosition().y_,
                         newM->GetNode()->GetPosition().z_);


    }
}


void NetworkActor::DebugDraw(const Color &color) {
    if (!vehicle_)
        return;

    DebugRenderer *dbgRenderer = GetScene()->GetComponent<DebugRenderer>();


    if (dbgRenderer) {

        // draw compound shape bounding box (the inertia bbox)
        Vector3 localExtents = vehicle_->GetRaycastVehicle()->GetCompoundLocalExtents();
        Vector3 localCenter = vehicle_->GetRaycastVehicle()->GetCompooundLocalExtentsCenter();
        BoundingBox bbox(-localExtents, localExtents);

        btTransform trans;
        vehicle_->GetRaycastVehicle()->getWorldTransform(trans);
        Vector3 posWS = ToVector3(trans.getOrigin());
        Vector3 centerWS = ToQuaternion(trans.getRotation()) * localCenter;
        posWS += centerWS;
        Matrix3x4 mat34(posWS, ToQuaternion(trans.getRotation()), 1.0f);

        /*
        dbgRenderer->AddBoundingBox(bbox, mat34, color);
        dbgRenderer->AddLine(posWS, posWS + node_->GetUp(), color);
        dbgRenderer->AddLine(posWS, posWS + node_->GetRight(), color);
*/

//        vehicle_->GetRaycastVehicle()->DrawDebugGeometry(dbgRenderer, false);

//        ToQuaternion(trans.getRotation()),
        // dbgRenderer->AddBoundingBox(bbox, mat34, color);
//        dbgRenderer->AddLine(posWS, posWS + node_->R, color);
        dbgRenderer->AddLine(posWS, posWS + this->vehicle_->GetNode()->GetDirection() * 40.0f, Color::CYAN);
        dbgRenderer->AddLine(posWS, toTarget_, Color::YELLOW);
    }
}