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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Network/Connection.h>
#include <Urho3D/Network/Network.h>
#include <Urho3D/Network/NetworkEvents.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/Log.h>

#include "Server.h"
#include "ClientObj.h"
#include "NetworkActor.h"
#include "CSP_Server.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
Server::Server(Context* context)
    : Object(context)
    , clientObjectID_(0)
{
    SubscribeToEvents();
}

Server::~Server()
{
}

void Server::RegisterClientHashAndScene(StringHash clientHash, Scene *scene)
{
    clientHash_ = clientHash;
    scene_ = scene;
}

void Server::InitializeScene(File &file)
{
    scene_->InstantiateXML(file, Vector3::ZERO, Quaternion());
    File saveFile(context_, GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Scenes/" + "MayaScape-ServerSceneInit.xml", FILE_WRITE);
    scene_->SaveXML(saveFile);
    URHO3D_LOGINFOF("InitializeScene: Scene checksum -> %s", ToStringHex(scene_->GetChecksum()).CString());
}

bool Server::StartServer(unsigned short port)
{
    return GetSubsystem<Network>()->StartServer(port);
}

bool Server::Connect(const String &addressRequet, unsigned short port, const VariantMap& identity)
{
    Network* network = GetSubsystem<Network>();
    String address = addressRequet.Trimmed();

    if (address.Empty())
    {
        address = "localhost"; // Use localhost to connect if nothing else specified
    }

    // Connect to server, specify scene to use as a client for replication
    clientObjectID_ = 0; // Reset own object ID from possible previous connection

    return network->Connect(address, port, scene_, identity);
}

void Server::Disconnect()
{
    Network* network = GetSubsystem<Network>();
    Connection* serverConnection = network->GetServerConnection();

    // If we were connected to server, disconnect. Or if we were running a server, stop it. In both cases clear the
    // scene of all replicated content, but let the local nodes & components (the static world + camera) stay
    if (serverConnection)
    {
        serverConnection->Disconnect();
        scene_->Clear(true, false);
        clientObjectID_ = 0;
        loginList_.Clear();
    }
    // Or if we were running a server, stop it
    else if (network->IsServerRunning())
    {
        network->StopServer();
        scene_->Clear(true, false);
    }
}

Node* Server::CreatePlayer(Connection* connection) {

    Node *playerNode = scene_->CreateChild("Player", REPLICATED);

    // Player is replaced with NetworkActor -> which is a Player
   // NetworkActor* actorClientObj_ = (NetworkActor*)playerNode->CreateComponent(clientHash_);

    // Store the player in map
    actorMap_[connection] =  new NetworkActor(context_);
    actorMap_[connection]->SetScene(scene_);
    actorMap_[connection]->Create(connection);

//    player_->GetNode()->SetPosition(Vector3(0, 0, 0));

    // If actor has a vehicle, snap the actor to vehicle
//    if (player_->vehicle_)
//        player_->GetNode()->SetPosition(player_->vehicle_->GetNode()->GetPosition());

//    player_->SetWaypoints(&waypointsWorld_);

    playerNode->SetRotation(Quaternion(0.0, -0.0, -0.0));

    // set identity
    if (connection)
    {

        String name = connection->identity_["UserName"].GetString();
        int colorIdx = connection->identity_["ColorIdx"].GetInt();
        actorMap_[connection]->SetClientInfo(name, colorIdx);

        URHO3D_LOGINFOF("client identity name=%s", name.CString());
        URHO3D_LOGINFOF("HandleClientConnected - data: [%s, %d]", name.CString(), colorIdx);
        // Store login name with connection
        loginList_.Populate(name.CString(), connection);
    }


    /*
    // Register player on CSP server
    auto csp = scene_->GetComponent<CSP_Server>();
    // Assign player node to csp snapshot
    csp->add_node(playerNode);
*/
    /*
    // Register player on CSP server
    auto csp = scene_->GetComponent<CSP_Server>();
    // Assign player node to csp snapshot
    csp->add_node(playerNode);*/

    return playerNode;
}

void Server::SubscribeToEvents()
{
    // Subscribe to network events
    SubscribeToEvent(E_SERVERCONNECTED, URHO3D_HANDLER(Server, HandleConnectionStatus));
    SubscribeToEvent(E_SERVERDISCONNECTED, URHO3D_HANDLER(Server, HandleConnectionStatus));
    SubscribeToEvent(E_CONNECTFAILED, URHO3D_HANDLER(Server, HandleConnectionStatus));
    SubscribeToEvent(E_CLIENTCONNECTED, URHO3D_HANDLER(Server, HandleClientConnected));
    SubscribeToEvent(E_CLIENTDISCONNECTED, URHO3D_HANDLER(Server, HandleClientDisconnected));
    // This is a custom event, sent from the server to the client. It tells the node ID of the object the client should control
    SubscribeToEvent(E_CLIENTOBJECTID, URHO3D_HANDLER(Server, HandleClientObjectID));
    // Events sent between client & server (remote events) must be explicitly registered or else they are not allowed to be received
    GetSubsystem<Network>()->RegisterRemoteEvent(E_CLIENTOBJECTID);
    // Additional events that we might be interested in
    SubscribeToEvent(E_CLIENTIDENTITY, URHO3D_HANDLER(Server, HandleClientIdentity));
    SubscribeToEvent(E_CLIENTSCENELOADED, URHO3D_HANDLER(Server, HandleClientSceneLoaded));
    SubscribeToEvent(E_NETWORKUPDATESENT, URHO3D_HANDLER(Server, HandleNetworkUpdateSent));
}



/*
void MayaScape::UpdateClientObjects()
{
    PODVector<Node*> playerNodes;
    scene_->GetNodesWithTag(playerNodes, "Player");
    auto clients = GetSubsystem<Network>()->GetClientConnections();
    for (auto it = clients.Begin(); it != clients.End(); ++it) {
        for (auto it2 = playerNodes.Begin(); it2 != playerNodes.End(); ++it2) {
            if ((*it2)->GetVar("GUID").GetString() == (*it)->GetGUID()) {
                if (!peers_[(*it)]) {
                    peers_[(*it)] = new Peer(context_);
                    peers_[(*it)]->SetConnection((*it));
                    peers_[(*it)]->SetScene(scene_);
                }
                peers_[(*it)]->SetNode((*it2));
            }
        }
    }
    for (auto it2 = playerNodes.Begin(); it2 != playerNodes.End(); ++it2) {
        if ((*it2)->GetVar("GUID").GetString() == GetSubsystem<Network>()->GetGUID()) {
            if (!peers_[GetSubsystem<Network>()->GetServerConnection()]) {
                peers_[GetSubsystem<Network>()->GetServerConnection()] = new Peer(context_);
                peers_[GetSubsystem<Network>()->GetServerConnection()]->SetConnection(GetSubsystem<Network>()->GetServerConnection());
                peers_[GetSubsystem<Network>()->GetServerConnection()]->SetScene(scene_);
            }
            peers_[GetSubsystem<Network>()->GetServerConnection()]->SetNode(*it2);
        }
    }
}
*/

void Server::UpdateClient(Connection* connection) {

    Network* network = GetSubsystem<Network>();

    // Connected to running server
//    if (network->GetServerConnection()->IsConnected()) {

        // Get the object this connection is controlling
        NetworkActor *actor = actorMap_[connection];

        if (actor) {
            actor->SetConnection(connection);
            actor->SetScene(scene_);
        }
  //  }
}

void Server::UpdateActors(float timeStep) {

    Network* network = GetSubsystem<Network>();

    // On running server
    if (network->IsServerRunning())
    {
        const Vector<SharedPtr<Connection> >& connections = network->GetClientConnections();

        for (unsigned i = 0; i < connections.Size(); ++i) {
            Connection *connection = connections[i];
//            const Controls& controls = connection->GetControls();

            // Get the object this connection is controlling
            NetworkActor *actor = actorMap_[connection];

            if (actor) {
                    actor->SetConnection(connection);
                    actor->SetScene(scene_);
                    // Apply update to actor
                    actor->FixedUpdate(timeStep);

                    // If actor has a vehicle, snap the actor to vehicle
                   // if (actor->vehicle_)
                   //     actor->GetNode()->SetPosition(actor->vehicle_->GetNode()->GetPosition());

                    // TODO: Add delay for player state?
//                    SendPlayerStateMsg(connection);

                    // Mark actor to update on network for replicate nodes
        //            actor->MarkNetworkUpdate();

            }
        }
    }
}

void Server::UpdatePhysicsPreStep(const Controls &controls)
{
    // This function is different on the client and server. The client collects controls (WASD controls + yaw angle)
    // and sets them to its server connection object, so that they will be sent to the server automatically at a
    // fixed rate, by default 30 FPS. The server will actually apply the controls (authoritative simulation.)
    Network* network = GetSubsystem<Network>();
    Connection* serverConnection = network->GetServerConnection();

//    URHO3D_LOGINFO("Server: UpdatePhysicsPreStep");


    // Client: collect controls
    if (serverConnection)
    {
  //      URHO3D_LOGINFO("Client: set controls for client sent to server");

        serverConnection->SetControls(controls);
    }
    // Server: apply controls to client objects
    else if (network->IsServerRunning())
    {
        const Vector<SharedPtr<Connection> >& connections = network->GetClientConnections();

        for (unsigned i = 0; i < connections.Size(); ++i)
        {
            Connection* connection = connections[i];
            const Controls& controls = connection->GetControls();
    //        URHO3D_LOGINFO("Server: connection->GetControls()");

            // Get the object this connection is controlling
            Node* clientNode = serverObjects_[connection];

            if (!clientNode)
                continue;

            ClientObj* clientObj = clientNode->GetDerivedComponent<ClientObj>();

            if (clientObj)
            {
//                URHO3D_LOGINFOF("Server: set controls for client [%d] -> %s", clientNode->GetID(), ToStringHex(controls.buttons_).CString());

                // TODO: Instead of applying controls right away - add to input buffer
                // Based on input buffer size, control client input demand

                clientObj->SetControls(controls);
                // Apply control to actor
                actorMap_[connection]->SetControls(controls);
                //clientObj->MarkNetworkUpdate();
            }
        }
    }
}

void Server::SendStatusMsg(StringHash msg)
{
    using namespace ServerStatus;

    // Send the event forward
    VariantMap& newEventData = GetEventDataMap();
    newEventData[P_STATUS] = msg;
    SendEvent(E_SERVERSTATUS, newEventData);
}


void Server::SendPlayerStateMsg(Connection* connection)
{
    NetworkActor* actor = actorMap_[connection];
    if (actor) {
        using namespace ClientPlayerState;

        // Send the event forward
        VariantMap &newEventData = GetEventDataMap();
        newEventData[P_ID] = actor->GetNode()->GetID();
        newEventData[P_VEHICLE_ID] = actor->GetVehicle()->GetNode()->GetID();
        newEventData[P_LIFE] = actor->GetLife();
        if (actor->GetVehicle()) {
            newEventData[P_RPM] = actor->GetVehicle()->GetCurrentRPM();
            newEventData[P_VELOCITY] = actor->GetVehicle()->GetSpeedKmH();
            newEventData[P_STEER] = actor->GetVehicle()->GetSteering();
        }
        //SendEvent(E_PLAYERSTATE, newEventData);
        // Finally send the object's node ID using a remote event
        connection->SendRemoteEvent(E_PLAYERSTATE, true, newEventData);

    }
}

void Server::HandleClientIdentity(StringHash eventType, VariantMap& eventData)
{
	using namespace ClientIdentity;
    URHO3D_LOGINFO("HandleClientIdentity");

    // When a client connects, assign to scene to begin scene replication
    Connection* newConnection = static_cast<Connection*>(eventData[P_CONNECTION].GetPtr());
    // Transmit scene from server to client
    newConnection->SetScene(scene_);

    URHO3D_LOGINFO("HandleClientIdentity - client assigned for scene replication.");
    URHO3D_LOGINFOF("Server: Scene checksum -> %s", ToStringHex(scene_->GetChecksum()).CString());

    // Create player for new client (NetworkActor is child of ClientObj)
    Node* clientObject = CreatePlayer(newConnection);
    serverObjects_[newConnection] = clientObject;

    URHO3D_LOGINFOF("Server: Client players -> %d", serverObjects_.Size());
    URHO3D_LOGINFOF("Server: scene replicated components -> %d", scene_->GetNumNetworkComponents());


    // Output the updated login list
    OutputLoginListToConsole();

//    scene_->MarkReplicationDirty(scene_->GetParent());

    // Finally send the object's node ID using a remote event
    VariantMap remoteEventData;
    remoteEventData[ClientObjectID::P_ID] = clientObject->GetID();
    newConnection->SendRemoteEvent(E_CLIENTOBJECTID, true, remoteEventData);

}

void Server::HandleClientSceneLoaded(StringHash eventType, VariantMap& eventData)
{
	using namespace ClientSceneLoaded;
    URHO3D_LOGINFO("HandleClientSceneLoaded");
    URHO3D_LOGINFOF("Server: Scene checksum -> %s", ToStringHex(scene_->GetChecksum()).CString());

}

bool first = true;
void Server::HandleNetworkUpdateSent(StringHash eventType, VariantMap& eventData)
{
//    URHO3D_LOGINFO("HandleNetworkUpdateSent");

    Network* network = GetSubsystem<Network>();
    Connection* serverConnection = network->GetServerConnection();

    // Client: collect controls
    if (serverConnection)
    {

        // On connected client
        if (serverConnection->IsConnected()) {

            // CLIENT CODE
            if (clientObjectID_) {

                Node *clientNode = scene_->GetChild(clientObjectID_);

                // TODO: Issue
                // Cannot retrieve replicated nodes on client
                if (clientNode) {

                    /*
                    if (first) {
                        File saveFile(context_,
                                      GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Scenes/MayaScape_FIRST.xml",
                                      FILE_WRITE);
                        scene_->SaveXML(saveFile);
                        first = false;
                    }*/

//                serverConnection->GetScene()->MarkNetworkUpdate();

                    //serverConnection->SetScene(scene_);

                    NetworkActor *networkActor = clientNode->GetDerivedComponent<NetworkActor>(true);

                    if (networkActor) {
                        networkActor->ClearControls();
                    }
                }
            }
        }
    } else {
        // On running server
        if (network->IsServerRunning()) {
           // URHO3D_LOGINFO("Server -> HandleNetworkUpdateSent");

//            ClientObj *clientObj = (ClientObj*)clientNode->CreateComponent(clientHash_);

            // Server: apply controls to client objects
            const Vector<SharedPtr<Connection> > &connections = network->GetClientConnections();

            for (unsigned i = 0; i < connections.Size(); ++i) {
                Connection *connection = connections[i];
                const Controls &controls = connection->GetControls();
//                URHO3D_LOGINFO("Server: connection->GetControls()");

                // Get the object this connection is controlling
                Node *clientNode = serverObjects_[connection];

                if (!clientNode)
                    continue;

                ClientObj *clientObj = clientNode->GetDerivedComponent<ClientObj>();

                if (clientObj) {
//                    URHO3D_LOGINFOF("Server: set controls for client [%d] -> %s", clientNode->GetID(), ToStringHex(controls.buttons_).CString());
                    clientObj->SetControls(controls);

                }
            }
        }
    }
}

void Server::HandleConnectionStatus(StringHash eventType, VariantMap& eventData)
{
    SendStatusMsg(eventType);
}

void Server::HandleClientConnected(StringHash eventType, VariantMap& eventData)
{
    using namespace ClientConnected;

    URHO3D_LOGINFO("HandleClientConnected");


}

void Server::DestroyPlayer(Connection* connection) {

    // NetworkActor nodes
    Node* object = serverObjects_[connection];

/*    NetworkActor* actor = actorMap_[connection];*/


    if (object)
    {
        URHO3D_LOGINFOF("**** DESTROYING CLIENT OBJECT -> %d", object->GetID());
//        object->RemoveAllChildren();
        object->Remove();
    }
/*
    if (actor)
    {
        actor->SetEnabled(false);
        URHO3D_LOGINFOF("**** DESTROYING CLIENT NETWORK ACTOR OBJECT -> %d", actor->GetID());
        actor->Remove();
    }
*/
    Vector<Connection*> connectList = loginList_.Values();

    URHO3D_LOGINFO("**** FINDING CONNECTION TO REMOVE FROM LOGIN LIST");

    for (int i = 0; i < connectList.Size(); i++) {
        if (connectList[i] == connection) {
            // Match
            URHO3D_LOGINFOF("**** MATCH: %s", loginList_.Keys()[i].CString());

            // Remove login from list
            loginList_.Erase(loginList_.Keys()[i]);
            break;
        }
    }

    serverObjects_.Erase(connection);
    actorMap_.Erase(connection);

    // Clear removed replicated nodes
    scene_->Clear(true, false);
//    scene_->MarkReplicationDirty(scene_);
}

void Server::HandleClientDisconnected(StringHash eventType, VariantMap& eventData)
{
    using namespace ClientConnected;
    URHO3D_LOGINFO("HandleClientDisconnected");

    // (On server)
    // This updates the login list by allowing a few cycles to update


    // When a client disconnects, remove the controlled object
    Connection* connection = static_cast<Connection*>(eventData[P_CONNECTION].GetPtr());
    // Destroy client player (network actor, vehicle, raycast vehicle)
    DestroyPlayer(connection);
    OutputLoginListToConsole();

    URHO3D_LOGINFO("**** HandleClientDisconnected COMPLETED");

}

void Server::HandleClientObjectID(StringHash eventType, VariantMap& eventData)
{
    URHO3D_LOGINFOF("HandleClientObjectID: clientID = %u", clientObjectID_);

    clientObjectID_ = eventData[ClientObjectID::P_ID].GetUInt();

/*


    // This function is different on the client and server. The client collects controls (WASD controls + yaw angle)
    // and sets them to its server connection object, so that they will be sent to the server automatically at a
    // fixed rate, by default 30 FPS. The server will actually apply the controls (authoritative simulation.)
    Network* network = GetSubsystem<Network>();
    Connection* serverConnection = network->GetServerConnection();

    // Server: apply controls to client objects
    if (network->IsServerRunning())
    {
        const Vector<SharedPtr<Connection> >& connections = network->GetClientConnections();

        for (unsigned i = 0; i < connections.Size(); ++i)
        {
            Connection* connection = connections[i];

            // Get the object this connection is controlling
            Node* clientNode = serverObjects_[connection];

            if (!clientNode)
                continue;

            clientNode->MarkNetworkUpdate();

            ClientObj* clientObj = clientNode->GetDerivedComponent<ClientObj>();
            if (clientObj)
            {
  //              clientObj->SetControls(controls);
            }
        }
    }*/
//    return GetSubsystem<Network>()->StartServer(port);

}

void Server::OutputLoginListToConsole() {

    URHO3D_LOGINFO("**** SERVER: CLIENT LIST *****************************************************");

    Vector<Connection*> connectList = loginList_.Values();

    for (int i = 0; i < connectList.Size(); i++) {
        URHO3D_LOGINFOF("**** LOGIN: %s", loginList_.Keys()[i].CString());
    }

    URHO3D_LOGINFO("******************************************************************************");

}