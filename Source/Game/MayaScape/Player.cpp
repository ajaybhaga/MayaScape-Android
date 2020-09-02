#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/IO/MemoryBuffer.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Urho2D/Sprite2D.h>
#include <Urho3D/Urho2D/CollisionBox2D.h>
#include <Urho3D/Urho2D/PhysicsWorld2D.h>
#include <Urho3D/Urho2D/PhysicsEvents2D.h>
#include <Urho3D/Urho2D/RigidBody2D.h>

#include <Urho3D/Network/Network.h>
#include <Urho3D/Network/NetworkEvents.h>

#include <Urho3D/Urho2D/Sprite2D.h>
#include <Urho3D/Urho2D/AnimatedSprite2D.h>
#include <Urho3D/Urho2D/AnimationSet2D.h>
#include <Urho3D/Graphics/DebugRenderer.h>

#include "Toolkit.h"
#include <SDL/SDL_log.h>
#include <Urho3D/DebugNew.h>
#include <Urho3D/Physics/PhysicsEvents.h>


#include "Player.h"
#include "RaycastVehicle.h"
#include "Vehicle.h"
#include "Missile.h"
#include "types.h"
#include "Bullet.h"
#include "AP.h"
#include "BUFF.h"
#include "HeatSource.h"
#include "Skill.h"
#include "Skill_Missile.h"
#include "Skill_Flare.h"
#include "Skill_Blink.h"

#define PI 3.1415926


Player::Player(Context* context) : GameObject(context)
{
	///Set original staus
	SetMaxHp(100.0f);
	SetHp(GetMaxHp());
	SetMass(10.0f);
	SetCollisionSize(Vector2(0.32f, 0.32f));
	SetInvincible(false);
	SetSpeed(0.0f);
	SetMaxSpeed(5.0f);
	SetDamping(0.015f);
	SetAcceleration(0.03f);
	SetBrake(0.05f);
	SetTowards(Vector3(0.0f, 1.0f, 0.0f));
	SetTurningVelocity(100.0f);
	SetBulletType("AP");
	///Set bullets type
	///Test bullets
	testcnt_ = 0;
	///4test
	nettest1209_ = 0;

    bulletType_ = "CB";
    lastFire_ = 0;
    targetAgentIndex_ = 0;
//    bulletType_ = "AP";
}

void Player::RegisterObject(Context* context)
{
	context->RegisterFactory<Player>();
}

//=============================================================================
// This function is called only from the main program when initially creating
// the vehicle, not on scene load
//=============================================================================
void Player::Init()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    Node* adjNode = GetScene()->CreateChild("AdjNode", LOCAL);
    adjNode->SetRotation(Quaternion(0.0, 0.0, -90.0f));

    // Init vehicle
    Node* vehicleNode = GetScene()->CreateChild("Vehicle", LOCAL);

    // Place on track
    vehicleNode->SetPosition(Vector3(-814.0f+Random(-400.f, 400.0f), 150.0f, -595.0f+Random(-400.f, 400.0f)));
    vehicleNode->SetEnabled(false);
    // Create the vehicle logic component
    vehicle_ = vehicleNode->CreateComponent<Vehicle>(LOCAL);
//    vehicle_->Init(false);
//    SetControls(ve)

    wpActiveIndex_ = 0;
    targetAgentIndex_ = 0;

    pRigidBody_ = vehicleNode->CreateComponent<RigidBody>(LOCAL);
//    pRigidBody_->SetMass(1.0f);
//    pRigidBody_->SetUseGravity(false);
 //   pRigidBody_->SetTrigger(true);


//    pRigidBody_ = vehicle_->GetRigidBody();

    // Store initial vehicle position as focus
//    focusObjects_.Push(vehicle_->GetNode()->GetPosition());

    // Create a directional light with cascaded shadow mapping
    vehicleHeadLamp_ = GetScene()->CreateChild("DirectionalLight");
    vehicleHeadLamp_->SetPosition(Vector3(vehicleNode->GetPosition().x_, 100.0f, vehicleNode->GetPosition().z_));
    vehicleHeadLamp_->SetDirection(Vector3(vehicleNode->GetRotation().x_, vehicleNode->GetRotation().y_, vehicleNode->GetRotation().z_));


    Light* light = vehicleHeadLamp_->CreateComponent<Light>();
    light->SetRadius(0.03f);
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetBrightness(0.24);
    light->SetCastShadows(true);
    //   light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
//    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    //  light->SetSpecularIntensity(0.01f);


    // Place on at corner of map
 //   TerrainPatch* p = terrain_->GetPatch(0, 0);
//    IntVector2 v = p->GetCoordinates();
//    vehicleNode->SetPosition(Vector3(v.x_-1400.0f, 40.0f, v.y_-1400.0f));
//    vehicleNode->SetPosition(Vector3(v.x_-1400.0f, 40.0f, v.y_+1400.0f));

    vehicleNode->SetRotation(Quaternion(0.0, -90.0, 0.0));


}

void Player::InitiateWeapons()
{
	///Initiate bullets

}

void Player::Start()
{
	Node* node = GetNode();
    // node collision
    SubscribeToEvent(GetNode(), E_NODECOLLISION, URHO3D_HANDLER(Player, HandlePlayerCollision));

	/*
	/// sprite
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	Sprite2D* sprite2d = cache->GetResource<Sprite2D>("Urho2D/Player.png");
	StaticSprite2D* staticsprite = node->CreateComponent<StaticSprite2D>();
	staticsprite->SetColor(Color(Random(1.0f), Random(1.0f), Random(1.0f)));
	staticsprite->SetSprite(sprite2d);
*/
	/// Add rigidbody
	/*rigibody2d_ = node->CreateComponent<RigidBody2D>(LOCAL);
	rigibody2d_->SetBodyType(BT_DYNAMIC);
	rigibody2d_->SetMass(GetMass());

	/// Add collisionbox
	collisionbox2d_ = node->CreateComponent<CollisionBox2D>(LOCAL);
	collisionbox2d_->SetSize(GetCollisionSize());
*/

/*	/// BUFF
	// HeatSource
	HeatSource* heatsource_ = node->CreateComponent<HeatSource>(LOCAL);
	heatsource_->SetOwner(SharedPtr<Node>(node));
	heatsource_->SetAttraction(3.0f);

	/// Skills
	// Missile
	Skill_Missile* skillMissile = node->CreateComponent<Skill_Missile>(LOCAL);
	skillMissile->SetTriggerKey(CTRL_Q);
	// Flare
	Skill_Flare* skillFlare = node->CreateComponent<Skill_Flare>(LOCAL);
	skillFlare->SetTriggerKey(CTRL_W);	
	// Blink
	Skill_Blink* skillBlink = node->CreateComponent<Skill_Blink>(LOCAL);
	skillBlink->SetTriggerKey(CTRL_E);
	/// Add some static buff to the Player
	//node->CreateComponent<HeatSource>();*/
}

void Player::Update(float timeStep)
{
	//nettest1209_++;
	//Toolkit::Print(String(nettest1209_));
}

void Player::FixedUpdate(float timeStep)
{
    lastFire_ += timeStep;

	/// Clients should not update the component on its own
	controls_ = vehicle_->controls_;


	Node* node = GetNode();
	///Acceleration
	if (controls_.IsDown(CTRL_UP)) {
		Accelerate();
	}
	///Damping
	else {
		Damping();
	}
	///Braking
	if (controls_.IsDown(CTRL_DOWN)) {
		Brake();
	}
	///Turn left
	if (controls_.IsDown(CTRL_LEFT)) {
		//Turn left
		//towards_ = Vector3(towards_.x_*cos(turningVelocity_*timeStep) - towards_.y_*sin(turningVelocity_*timeStep), towards_.x_*sin(turningVelocity_*timeStep) + towards_.y_*cos(turningVelocity_*timeStep), 0.0f);		
		node->Rotate2D(turningVelocity_*timeStep);
		// The angle between rotation2d and x-axis
		float angle = 90.0f + node->GetRotation2D();
		// The towards vector according to the angle
		towards_ = Vector3(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f), 0.0f);
	}
	///Turn right
	if (controls_.IsDown(CTRL_RIGHT)) {
		//Turn right
		//towards_ = Vector3(towards_.x_*cos(turningVelocity_*timeStep) + towards_.y_*sin(turningVelocity_*timeStep), -towards_.x_*sin(turningVelocity_*timeStep) + towards_.y_*cos(turningVelocity_*timeStep), 0.0f);	
		node->Rotate2D(-turningVelocity_*timeStep);
		// The angle between rotation2d and x-axis
		float angle = 90.0f + node->GetRotation2D();
		// The towards vector according to the angle
		towards_ = Vector3(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f), 0.0f);
	}



	///Rotate the Player
	//node->SetRotation(Quaternion(Vector3(0.0f, 1.0f, 0.0f), towards_));

	///Impulse the Player(may be not true)
	//rigibody2d_->SetLinearVelocity(Vector2(towards_.x_, towards_.y_).Normalized() * speed_);

	/// If the Player'hp <=0 ,then destroy it.
	if (GetHp() <= 0.0f) this->Destroyed();

	if (toTarget_ != Vector3::ZERO) {
        // Only pass once rigid body is setup
        if (pRigidBody_) {
            // Compute steer force
            ComputeSteerForce();
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


                if (autoSteering_) {
                    URHO3D_LOGINFOF("***** Player AUTO-STEERING ENABLED - Vehicle Steer [%f]", vehicle_->GetSteering());
                    URHO3D_LOGINFOF("***** Player - waypoint [%d]=[%f,%f,%f,%f]", wpActiveIndex_,
                                    waypoints_->At(wpActiveIndex_).x_, waypoints_->At(wpActiveIndex_).y_,
                                    waypoints_->At(wpActiveIndex_).z_, steering);
                    URHO3D_LOGINFOF("***** Player - target =[%f,%f,%f]", toTarget_.x_, toTarget_.y_, toTarget_.z_);

                    // Enable auto-steering
                    vehicle_->UpdateSteering(steering);
                }
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
            }*/
            }


        }
    }
}


void Player::HandlePlayerCollision(StringHash eventType, VariantMap & eventData)
{
    using namespace NodeCollision;

    // decrease health if player player collides with boids
    Node* collidedNode = static_cast<Node*>(eventData[P_OTHERNODE].GetPtr());

    URHO3D_LOGDEBUGF("Player::HandlePlayerCollision() -> collidedNode: [%s]", collidedNode->GetName().CString());

    if (collidedNode->GetName() == "boid")
    {
        // engine_->Exit();
        printf("Player collided with Boid!! \n");

    }
}

void Player::Fire() {
    Fire(toTarget_);
}

void Player::Fire(Vector3 target)
{
    node_ = GetNode();
	Scene* scene = GetScene();


    URHO3D_LOGDEBUG("Player::Fire()");

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
        testcnt_++;

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
        Node* bullet0 = scene->CreateChild("bullet", REPLICATED);
        Missile* newM = bullet0->CreateComponent<Missile>(LOCAL);
        newM->SetProducer(vehicle_->GetNode()->GetID());


        // Store local missile list
        //missileList_.Push(vehicle_->GetNode()->GetID());

//        VariantMap& eventData = GetNode()->GetEventDataMap();
//        eventData["owner"] = SharedPtr<Player>(this);

//        eventData["missileOwner"] = this->GetID();
  //       vehicle_->SendEvent(E_NODECOLLISION, eventData);



        // Set the ownership of the bullet to the Player
//        bullet0->GetComponent<Missile>()->SetProducer(SharedPtr<Node>(vehicle_->GetNode()));

        Node* tgt = scene->CreateChild("missileTarget", LOCAL);
        //tgt->>SetPosition(0f,0f,0f);
        tgt->SetPosition(target);
        newM->AddTarget(SharedPtr<Node>(tgt));
        // Assign the producer node
        newM->AssignProducer(vehicle_->GetNode()->GetID(), vehicle_->GetNode()->GetPosition()+Vector3(0.0f, 2.0f, 0.0f));
        URHO3D_LOGDEBUGF("Player::Fire() [%d] -> %d, [%f,%f,%f]", vehicle_->GetNode()->GetID(), testcnt_, newM->GetNode()->GetPosition().x_,
                         newM->GetNode()->GetPosition().y_,
                         newM->GetNode()->GetPosition().z_);


    }
}

void Player::Destroyed()
{
	Node* node = GetNode();
	node->Remove();
}

void Player::ComputeSteerForce() {

    //set the force member variable to zero
    force_ = Vector3(0, 0, 0);

    if (!waypoints_)
        return;

    if ((!pRigidBody_) || (waypoints_->Empty())) {
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
        toTarget = pRigidBody_->GetPosition() - waypoints_->At(wpActiveIndex_);
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

    force_ += (desiredVelocity - pRigidBody_->GetLinearVelocity());

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


void Player::DebugDraw(const Color &color)
{
    if (!vehicle_)
        return;

    DebugRenderer *dbgRenderer = GetScene()->GetComponent<DebugRenderer>();
    node_ = GetNode();


    if ( dbgRenderer )
    {

        // draw compound shape bounding box (the inertia bbox)
        Vector3 localExtents = vehicle_->GetRaycastVehicle()->GetCompoundLocalExtents();
        Vector3 localCenter  = vehicle_->GetRaycastVehicle()->GetCompooundLocalExtentsCenter();
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
        dbgRenderer->AddLine(posWS, posWS + this->vehicle_->GetNode()->GetDirection()*40.0f, Color::CYAN);
        dbgRenderer->AddLine(posWS, toTarget_, Color::YELLOW);





    }
}


