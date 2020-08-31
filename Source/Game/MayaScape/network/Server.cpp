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

Node* Server::CreateClientObject(Connection *connection)
{
    Node* clientNode = scene_->CreateChild("client");
//    clientNode->SetPosition(Vector3(Random(40.0f) - 20.0f, 300.0f, Random(40.0f) - 20.0f));
    clientNode->SetPosition(Vector3(0, 0.0f, 0));

    ClientObj *clientObj = (ClientObj*)clientNode->CreateComponent(clientHash_);

    // set identity
    if (connection)
    {
        String name = connection->identity_["UserName"].GetString();
        int colorIdx = connection->identity_["ColorIdx"].GetInt();
        clientObj->SetClientInfo(name, colorIdx);

        URHO3D_LOGINFOF("client identity name=%s", name.CString());
        URHO3D_LOGINFOF("HandleClientConnected - data: [%s, %d]", name.CString(), colorIdx);
        // Store login name with connection
        loginList_.Populate(name.CString(), connection);
    }

    return clientNode;
}


void Server::CreatePlayer(Connection* connection) {
    Node *playerNode = scene_->CreateChild("Player", LOCAL);

    playerNode->SetPosition(Vector3(0, 0, 0));

    // Place on track
//    playerNode->SetPosition(Vector3(-814.0f + Random(-400.f, 400.0f), 400.0f, -595.0f + Random(-400.f, 400.0f)));

    // Player is replaced with NetworkActor -> which is a Player
    NetworkActor* player_ = playerNode->CreateComponent<NetworkActor>(LOCAL);
    player_->isServer_ = true;

    // Store the player in map
    actorMap_.Populate(connection, player_);

    player_->GetNode()->SetPosition(Vector3(0, 0, 0));

//    player_->SetWaypoints(&waypointsWorld_);

    playerNode->SetRotation(Quaternion(0.0, -0.0, -0.0));


    // Register player on CSP server
    auto csp = scene_->GetComponent<CSP_Server>();
    // Assign player node to csp snapshot
    csp->add_node(playerNode);

    /*
    // Register player on CSP server
    auto csp = scene_->GetComponent<CSP_Server>();
    // Assign player node to csp snapshot
    csp->add_node(playerNode);*/

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

void Server::UpdateActors(float timeStep) {

    Network* network = GetSubsystem<Network>();

    // On running server
    if (network->IsServerRunning())
    {
        const Vector<SharedPtr<Connection> >& connections = network->GetClientConnections();

        for (unsigned i = 0; i < connections.Size(); ++i)
        {
            Connection* connection = connections[i];
//            const Controls& controls = connection->GetControls();

            // Get the object this connection is controlling
            NetworkActor* actor = actorMap_[connection];

            if (actor) {
                // Apply update to actor
                actor->FixedUpdate(timeStep);
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
                URHO3D_LOGINFOF("Server: set controls for client [%d] -> %s", clientNode->GetID(), ToStringHex(controls.buttons_).CString());

                clientObj->SetControls(controls);
                // Apply control to actor
                actorMap_[connection]->SetControls(controls);
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

void Server::HandleClientIdentity(StringHash eventType, VariantMap& eventData)
{
	using namespace ClientIdentity;
    URHO3D_LOGINFO("HandleClientIdentity");

    // When a client connects, assign to scene to begin scene replication
    Connection* newConnection = static_cast<Connection*>(eventData[P_CONNECTION].GetPtr());
    newConnection->SetScene(scene_);

    URHO3D_LOGINFO("HandleClientIdentity - client assigned for scene replication.");
    URHO3D_LOGINFOF("Server: Scene checksum -> %s", ToStringHex(scene_->GetChecksum()).CString());


    // Then create a controllable object for that client
    Node* clientObject = CreateClientObject(newConnection);
    serverObjects_[newConnection] = clientObject;

    // Create player for new client
    CreatePlayer(newConnection);

    // Output the updated login list
    OutputLoginListToConsole();

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

void Server::HandleNetworkUpdateSent(StringHash eventType, VariantMap& eventData)
{
//    URHO3D_LOGINFO("HandleNetworkUpdateSent");

    Network* network = GetSubsystem<Network>();
    Connection* serverConnection = network->GetServerConnection();

    // Client: collect controls
    if (serverConnection)
    {
        if (clientObjectID_)
        {

            Node *clientNode = scene_->GetChild(clientObjectID_);

            if (clientNode)
            {

                ClientObj *clientObj = clientNode->GetDerivedComponent<ClientObj>();

                if (clientObj)
                {
                    clientObj->ClearControls();
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

void Server::HandleClientDisconnected(StringHash eventType, VariantMap& eventData)
{
    using namespace ClientConnected;
    URHO3D_LOGINFO("HandleClientDisconnected");

    // (On server)
    // This updates the login list by allowing a few cycles to update

    // When a client disconnects, remove the controlled object
    Connection* connection = static_cast<Connection*>(eventData[P_CONNECTION].GetPtr());
    Node* object = serverObjects_[connection];
    if (object)
    {
        URHO3D_LOGINFOF("**** DESTROYING CLIENT NETWORK OBJECT -> %d", object->GetID());
        object->Remove();
    }


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