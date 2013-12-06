#ifndef ReMod3D_Project3_Control_Solution_h
#define ReMod3D_Project3_Control_Solution_h

/*
 * Keyboard Map
 * See Camera.h for detailed camera control information
 * Buttons 1,2,3,4,5,6,w,a,s,d,(up arrow),(down arrow),(left arrow),(right arrow) control the camera
 * Holding down the left mouse button and moving the mouse manipulates the orientation of the camera
 * Button p sets program pause state (pause/unpause)
 * Button f switches between Wheelbot view and global view (NOTE: Color blob sensing will not work in global view, and the camera cannot be manually
 * moved in Wheelbot view)
 */


////PROJECT 3, LEVEL 1, Change number of wumpuses and number of pits in Project3.cpp to 0
////PROJECT 3, LEVEL 2, Change number of wumpuses to 0 and number of pits to > 0 in Project3.cpp
////PROJECT 3, LEVEL 3, Change number of wumpuses to > 0 and number of pits to > 0 in Project3.cpp.
////PROJECT 3, LEVEL 4, Change number of wumpuses to > 0 and number of pits to > 0 and pass true for shootingNoise in Project3.cpp
////PROJECT 3, POSSIBLE LEVEL 4, TBD


/* ----------------------------------------- DO NOT MODIFY ------------------------------------------------------*/
#include "ModuleProgram.h"
#include "WheelbotModule.h"
#include "SimulationUtility.h"
#include "ActionLog.h"
#include <map>
#include <queue>
#include <utility>
#include <stack>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/math/distributions/normal.hpp>
using namespace std;

//Possible Wheelbot actions
enum WheelbotActions {
    ACTION_FORWARD = 0,
    ACTION_BACKWARD,
    ACTION_LEFT,
    ACTION_RIGHT,
    ACTION_STOP,
    ACTION_SHOOT
};

//Structure for storing Wumpus information
struct Wumpus {
    const char *name;
    int index;
};

//Data structure for rendering laser shooting
struct RenderLine {
    PxVec3 p0;
    PxVec3 p1;
    int lifeTime;
    
};

//Graph data structure
class Graph {
public:
    vector<PxVec3> graphPoints;
    vector<pair<int, int> > graphEdges;
    Graph(vector<PxVec3> p, vector<pair<int, int> >  e) {
        this->graphPoints = p;
        this->graphEdges = e;
    }
    Graph(){}
};

//Node representing a search node in A*
class Node {
public:
    //Give each node an integer to represent the state
    int state;
    //Pointer to the parent (for reconstructing solution path)
    Node *parent;
    int parentState;
    //Actions available to this node
    vector<int> actions;
    int currentAction;
    //Action directions
    vector<PxVec2> actionDirections;
    //Heuristic value, cost along optimal path, and fVal (adds heuristic value and cost)
    double hVal;
    double fVal;
    double gVal;
    
    //Constructor
    Node (int s, Node * p, vector<int> a, double h, double g) {
        this->state = s;
        this->parent = p;
        this->actions = a;
        this->hVal = h;
        this->gVal = g;
        this->fVal = h + g;
    }
    Node (int s, int p, vector<PxVec2> a, double h, double g) {
        this->state = s;
        this->parentState = p;
        this->actionDirections = a;
        this->hVal = h;
        this->gVal = g;
        this->fVal = h + g;
        this->currentAction = 0;
    }
};


//Compares two Node fVals. We want lower fVals at the front of the priority queue
class CompareNode {
public:
    bool operator()(Node * n1, Node * n2)
    {
        if (n1->fVal > n2->fVal) return true;
        return false;
    }
};

class Project3_Control:rm3d::ai::ModuleProgram<rm3d::ai::WheelbotDock *> {
private:
    //Simulator base object and Wheelbot world model
    WheelbotSimBase* simulator;
    rm3d::ai::WheelbotModel *model;
    //Obstacle material
    PxMaterial *obMaterial;
    //Radius around goal specifying when we have arrived at a goal
    static PxReal currentGoalRadius;
    //Magnitude of wheel velocities for movements
    PxReal velocityMagnitude;
    //start node
    static int start;
    //Minimum distance between nodes
    static float minDistance;
    //goal node
    static int goal;
    //Input Graph
    static Graph g;
    //Learned Graph, initially empty
    static Graph learnedG;
    //Current position of Wheelbot
    static PxVec3 currentWheelbotPosition;
    //A plan of points to visit
    static vector<int> plan;
    //Pits in the graph
    static vector<int> graphPits;
    //Actions from each state
    static vector<vector<int> > actions;
    //Winds associated with each edge
    static vector<float> graphWinds;
    //Wumpuses on the graph
    static vector<Wumpus> graphWumpuses;
    //Smells associated with each edge
    static vector<float> graphSmell;
    //vector of all the posible lines
    stack< pair<int,PxVec2> > actionsAtNode;
    //vector for winds info
    stack< pair<float,float> > infoAtNode;
    //vector for smell info
    stack< float > smellsAtNode;
    //vector for smell Node
    vector<bool> windsNodes;
    //current goal position
    PxVec2 currentAction;
    //current node;
    PxVec3 previousNode;
    //Astar solution
    vector<Node *> solution;
    //get back flag;
    bool getBack;
    //want Back
    bool Shoot;
    //Shot distanc
    float shootDistance;
    //current position
    int previousPos;
    //Currently adding noise to shooting
    static bool shootingNoise;
    //Distribution for shoot action sampling
    static boost::mt19937 rngShoot;
    static boost::normal_distribution<> shootDistribution;
    static boost::variate_generator< boost::mt19937, boost::normal_distribution<> > shootResampler;
    static vector<RenderLine> renderLines;
public:
    
    
    //Helpful rendering of paths, map nodes, pits, wumpuses, smells, and winds, as well as the learned map
    static void render() {
        //Render raycasts for a certain number of iterations (otherwise, they are just blips and hard to see)
        vector<int> deleteIndices;
        for (int i = 0; i<renderLines.size(); i++) {
            if (renderLines[i].lifeTime > 0) {
                rm3d::Renderer::DrawLine(renderLines[i].p0, renderLines[i].p1, rm3d::Color(0,0,1));
                renderLines[i].lifeTime = renderLines[i].lifeTime - 1;
            } else {
                deleteIndices.push_back(i);
            }
        }
        
        int sizeRLines = renderLines.size();
        for (int i=0; i<deleteIndices.size(); i++) {
            renderLines.erase(renderLines.begin() + deleteIndices[i]);
        }
        
        renderLines.resize(sizeRLines - deleteIndices.size());
        for (int i=0; i<g.graphPoints.size(); i++) {
            bool isPit = false;
            bool isWumpus = false;
            for (int j=0; j<graphPits.size(); j++) {
                if (graphPits[j] == i) {
                    isPit = true;
                    break;
                }
            }
            for (int j=0; j<graphWumpuses.size(); j++) {
                if (graphWumpuses[j].index == i) {
                    isWumpus = true;
                    break;
                }
            }
            
            
            //Draw the map nodes with colors corresponding to regular node (blue), pit (pink), wumpus (black) and current node (light blue)
            if ((currentWheelbotPosition - g.graphPoints[i]).magnitude() <= currentGoalRadius + 0.05) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,1,1));
            } else if (isPit) {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(148.0/256.0,0,211.0/256.0));
            } else if (isWumpus){
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,0,0));
            } else {
                rm3d::Renderer::DrawSphere(PxTransform(g.graphPoints[i]), 0.2, rm3d::Color(0,0,1));
            }
            
        }
        
        //Draw the map edges (smelly edges black, pit edges red)
        for (int j=0; j<g.graphEdges.size(); j++) {
            if (graphWinds[j] > 0.0) {
                rm3d::Renderer::DrawLine(PxVec3(g.graphPoints[g.graphEdges[j].first].x,
                                                g.graphPoints[g.graphEdges[j].first].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].first].z),
                                         PxVec3(g.graphPoints[g.graphEdges[j].second].x,
                                                g.graphPoints[g.graphEdges[j].second].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].second].z), rm3d::Color(1,0,0));
            } else if (graphSmell[j] > 0.0) {
                rm3d::Renderer::DrawLine(PxVec3(g.graphPoints[g.graphEdges[j].first].x,
                                                g.graphPoints[g.graphEdges[j].first].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].first].z),
                                         PxVec3(g.graphPoints[g.graphEdges[j].second].x,
                                                g.graphPoints[g.graphEdges[j].second].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].second].z), rm3d::Color(0,0,0));
            } else {
                rm3d::Renderer::DrawLine(PxVec3(g.graphPoints[g.graphEdges[j].first].x,
                                                g.graphPoints[g.graphEdges[j].first].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].first].z),
                                         PxVec3(g.graphPoints[g.graphEdges[j].second].x,
                                                g.graphPoints[g.graphEdges[j].second].y + 0.05,
                                                g.graphPoints[g.graphEdges[j].second].z), rm3d::Color(0,1,1));
            }
        }
        
        //Draw the learned map nodes and edges in a similar way
        for (int i=0; i<learnedG.graphPoints.size(); i++) {
            if ((currentWheelbotPosition - learnedG.graphPoints[i]).magnitude() <= currentGoalRadius + 0.05) {
                rm3d::Renderer::DrawSphere(PxTransform(PxVec3(learnedG.graphPoints[i].x,
                                                              learnedG.graphPoints[i].y + 0.05,
                                                              learnedG.graphPoints[i].z)), 0.2, rm3d::Color(1,1,0));
            } else {
                rm3d::Renderer::DrawSphere(PxTransform(PxVec3(learnedG.graphPoints[i].x,
                                                              learnedG.graphPoints[i].y + 0.05,
                                                              learnedG.graphPoints[i].z)), 0.2, rm3d::Color(1,0,0));
            }
            
        }
        for (int j=0; j<learnedG.graphEdges.size(); j++) {
            rm3d::Renderer::DrawLine(PxVec3(learnedG.graphPoints[learnedG.graphEdges[j].first].x,
                                            learnedG.graphPoints[learnedG.graphEdges[j].first].y + 0.1,
                                            learnedG.graphPoints[learnedG.graphEdges[j].first].z),
                                     PxVec3(learnedG.graphPoints[learnedG.graphEdges[j].second].x,
                                            learnedG.graphPoints[learnedG.graphEdges[j].second].y + 0.1,
                                            learnedG.graphPoints[learnedG.graphEdges[j].second].z), rm3d::Color(0,1,0));
        }
        
    }
    
    //Constructor
    Project3_Control(WheelbotSimBase* simulator, Graph graph, int startNode, int goalNode, vector<int> pits, vector<float> winds,
                     vector<Wumpus> wumpuses, vector<float> smell, bool shootNoise, float minDistanceBetween) {
        graphPits = pits;
        graphWinds = winds;
        graphSmell = smell;
        shootingNoise = shootNoise;
        graphWumpuses = wumpuses;
        minDistance = minDistanceBetween;
        obMaterial = simulator->getSimulationPhysics()->createMaterial(0.5, 0.5, 0.5);
        this->programCounter = 0;
        this->actionQueue = new rm3d::ai::ActionLog*[4];
        this->simulator = simulator;
        this->actionQueue[0] = NULL;
        this->actionQueue[1] = NULL;
        this->actionQueue[2] = NULL;
        this->actionQueue[3] = NULL;
        this->velocityMagnitude = 15.0;
        this->getBack=false;
        this->Shoot=false;
        start = startNode;
        goal = goalNode;
        g = graph;
        //Compute actions for each state
        for (int i=0; i<g.graphPoints.size(); i++) {
            actions.push_back(vector<int>());
            for (int j=0; j<graph.graphEdges.size(); j++) {
                if (g.graphEdges[j].first == i) {
                    actions[i].push_back(g.graphEdges[j].second);
                }
            }
        }
        
    }
    
    ~Project3_Control() {
        if (this->actionQueue[0] != NULL) delete this->actionQueue[0];
        if (this->actionQueue[1] != NULL) delete this->actionQueue[1];
        if (this->actionQueue[2] != NULL) delete this->actionQueue[2];
        if (this->actionQueue[3] != NULL) delete this->actionQueue[3];
        delete actionQueue;
    }
    
    rm3d::ai::ActionLog **getActionQueue() {
        return this->actionQueue;
    }
    rm3d::ai::Message<rm3d::ai::WheelbotDock*> **getMessageQueue() {
        return this->messageQueue;
    }
    rm3d::ai::RangedMessage** getRangedMessageQueue() {
        return this->rangedMessageQueue;
    }
    int getNumberOfRangedMessages() {
        return this->numRangedMessages;
    }
    int getNumberOfActions() {
        return this->numActions;
    }
    int getNumberOfMessages() {
        return this->numMessages;
    }
    
    //Sensor for current wheelbot pose
    PxTransform getCurrentWheelbotPose() {
        return boost::any_cast<PxTransform>(this->model->getCurrentState()->sensorValues[rm3d::ai::WheelbotModule::posOrientInt]);
    }
    
    //Sensor indicating whether or not we are at a node. If we are not at a node, we must be on an edge somewhere between nodes
    bool atANode() {
        for (int i=0; i<g.graphPoints.size(); i++) {
            if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                return true;
            }
        }
        return false;
    }
    
    //Returns the current node point
    PxVec3 getCurrentNode() {
        for (int i=0; i<g.graphPoints.size(); i++) {
            if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                return g.graphPoints[i];
            }
        }
        return PxVec3(-100000);
    }
    
    //Returns the actions (2D direction vectors) available to our robot at each node
    vector<PxVec2> getActionVectorsForCurrentNode() {
        vector<PxVec2> currentActions;
        bool atNode = atANode();
        if (atNode) {
            int node = -1;
            for (int i=0; i<g.graphPoints.size(); i++) {
                if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                    node = i;
                }
            }
            vector<int> actionsForNode = actions[node];
            for (int i=0; i<actionsForNode.size(); i++) {
                PxVec2 vec = PxVec2(g.graphPoints[actionsForNode[i]].x - getCurrentWheelbotPose().p.x,
                                    g.graphPoints[actionsForNode[i]].z - getCurrentWheelbotPose().p.z);
                vec.normalize();
                currentActions.push_back(vec);
                
            }
        }
        return currentActions;
    }
    
    //Returns the vector corresponding to the  given action at the given node in the learned graph
    PxVec2 getVectorForActionAtNode(int stateInLearnedGraph, int action) {
        PxVec2 actionVec;
        int node = -1;
        for (int i=0; i<g.graphPoints.size(); i++) {
            if ((learnedG.graphPoints[stateInLearnedGraph] - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                node = i;
            }
        }
        
        if (action == -1) {
            PxVec3 vec(g.graphPoints[node] - getCurrentWheelbotPose().p);
            actionVec = PxVec2(vec.x, vec.z);
        } else {
            PxVec3 vec(g.graphPoints[actions[node][action]] - getCurrentWheelbotPose().p);
            actionVec = PxVec2(vec.x, vec.z);
        }
        return actionVec;
    }
    
    //Returns the winds associated with the actions available at the current node
    vector<float> getCurrentWinds() {
        vector<float> currentWinds;
        bool atNode = atANode();
        if (atNode) {
            int node = -1;
            for (int i=0; i<g.graphPoints.size(); i++) {
                if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                    node = i;
                }
            }
            vector<int> actionsForNode = actions[node];
            for (int i=0; i<actionsForNode.size(); i++) {
                int edge = -1;
                for (int j=0; j<g.graphEdges.size(); j++) {
                    if (g.graphEdges[j].first == node && g.graphEdges[j].second == actionsForNode[i]) {
                        edge = j;
                    }
                }
                currentWinds.push_back(graphWinds[edge]);
                
            }
        }
        return currentWinds;
    }
    
    //Returns the smells associated with the actions available at the current node
    vector<float> getCurrentSmells() {
        vector<float> currentSmells;
        bool atNode = atANode();
        if (atNode) {
            int node = -1;
            for (int i=0; i<g.graphPoints.size(); i++) {
                if ((getCurrentWheelbotPose().p - g.graphPoints[i]).magnitude() <= currentGoalRadius) {
                    node = i;
                }
            }
            vector<int> actionsForNode = actions[node];
            for (int i=0; i<actionsForNode.size(); i++) {
                int edge = -1;
                for (int j=0; j<g.graphEdges.size(); j++) {
                    if (g.graphEdges[j].first == node && g.graphEdges[j].second == actionsForNode[i]) {
                        edge = j;
                    }
                }
                currentSmells.push_back(graphSmell[edge]);
                
            }
        }
        return currentSmells;
    }
    
    //If we have killed the Wumpus
    bool updateGraphOnWumpusDeath(Wumpus w) {
        vector<Wumpus> ws;
        for (int i=0; i<graphWumpuses.size(); i++) {
            if (graphWumpuses[i].index != w.index) {
                ws.push_back(graphWumpuses[i]);
            }
        }
        graphWumpuses.clear();
        graphWumpuses = ws;
        vector<int> actionsForNode = actions[w.index];
        for (int i=0; i<actionsForNode.size(); i++) {
            int edge = -1;
            for (int j=0; j<g.graphEdges.size(); j++) {
                if ((g.graphEdges[j].first == w.index && g.graphEdges[j].second == actionsForNode[i]) ||
                    (g.graphEdges[j].second == w.index && g.graphEdges[j].first == actionsForNode[i])) {
                    graphSmell[j] = 0.0;
                }
            }
            
        }
        for (int j =0; j<graphWumpuses.size(); j++) {
            for (int i=0; i<g.graphEdges.size(); i++) {
                if (g.graphEdges[i].first == graphWumpuses[j].index) {
                    graphSmell[i] = (g.graphPoints[g.graphEdges[i].first] - g.graphPoints[g.graphEdges[i].second]).magnitude();
                } else if (g.graphEdges[i].second == graphWumpuses[j].index) {
                    graphSmell[i] = (g.graphPoints[g.graphEdges[i].second] - g.graphPoints[g.graphEdges[i].first]).magnitude();
                }
            }
        }
    }
    
    //Make Wumpus run away
    bool updateGraphOnWumpusFlee(Wumpus w) {
        vector<int> actionsForNode = actions[w.index];
        for (int i=0; i<actionsForNode.size(); i++) {
            int edge = -1;
            for (int j=0; j<g.graphEdges.size(); j++) {
                if ((g.graphEdges[j].first == w.index && g.graphEdges[j].second == actionsForNode[i]) ||
                    (g.graphEdges[j].second == w.index && g.graphEdges[j].first == actionsForNode[i])) {
                    graphSmell[j] = 0.0;
                }
            }
            
        }
        
        bool okNode = false;
        int itCounter = 0;
        int newIndex = -1;
        while (!okNode && itCounter < actionsForNode.size()) {
            okNode = true;
            newIndex = actionsForNode[rand() % actionsForNode.size()];
            
            for (int i=0; i<graphPits.size(); i++) {
                if (newIndex == graphPits[i]) {
                    okNode = false;
                }
            }
            
            for (int i=0; i<graphWumpuses.size(); i++) {
                if (newIndex == graphWumpuses[i].index) {
                    okNode = false;
                }
            }
            
            if ((g.graphPoints[newIndex] - getCurrentWheelbotPose().p).magnitude() < minDistance) {
                okNode = false;
            }
            
            itCounter++;
            
        }
        
        if (okNode) {
            cout<<"Moving From "<<w.index<<" to "<<newIndex<<endl;
            for (int i=0; i<graphWumpuses.size(); i++) {
                if (graphWumpuses[i].index == w.index) {
                    graphWumpuses[i].index = newIndex;
                }
            }
            
            vector<rm3d::ai::Obstacle> obs = simulator->getObstacles();
            for (int i=0; i<obs.size(); i++) {
                if (strcmp(obs[i].obstacleBody->getName(), w.name) == 0) {
                    PxRigidDynamic *rd = (PxRigidDynamic *)obs[i].obstacleBody;
                    rd->setGlobalPose(PxTransform(PxVec3(g.graphPoints[newIndex]))*PxTransform(PxVec3(0,1,0)));
                }
            }
        }
        
        for (int j =0; j<graphWumpuses.size(); j++) {
            for (int i=0; i<g.graphEdges.size(); i++) {
                if (g.graphEdges[i].first == graphWumpuses[j].index) {
                    graphSmell[i] = (g.graphPoints[g.graphEdges[i].first] - g.graphPoints[g.graphEdges[i].second]).magnitude();
                } else if (g.graphEdges[i].second == graphWumpuses[j].index) {
                    graphSmell[i] = (g.graphPoints[g.graphEdges[i].second] - g.graphPoints[g.graphEdges[i].first]).magnitude();
                }
            }
        }
    }
    
    //Explode robot if we reach too close to a pit or a Wumpus
    void checkForFailure() {
        bool fail = false;
        for (int i=0; i<graphPits.size(); i++) {
            if ((getCurrentWheelbotPose().p - g.graphPoints[graphPits[i]]).magnitude() <= currentGoalRadius) {
                fail = true;
            }
        }
        
        for (int i=0; i<graphWumpuses.size(); i++) {
            if ((getCurrentWheelbotPose().p - g.graphPoints[graphWumpuses[i].index]).magnitude() <= currentGoalRadius) {
                fail = true;
            }
        }
        
        if (fail) {
            rm3d::ai::WheelbotModule * wm = (rm3d::ai::WheelbotModule *)simulator->getModuleAtIndex(0);
            cout<<"Boom"<<endl;
            wm->getBody()->addForce(PxVec3(50,100,0));
        }
    }
    
    //Wheelbot action that can be taken.
    void takeAction(WheelbotActions action, PxVec2 direction = PxVec2(1,0), float distance = 1.0) {
        this->numActions = 4;
        if (action == ACTION_BACKWARD) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)velocityMagnitude);
        } else if (action == ACTION_FORWARD) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)-velocityMagnitude);
        } else if (action == ACTION_LEFT) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)-velocityMagnitude);
        } else if (action == ACTION_RIGHT) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)velocityMagnitude);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)-velocityMagnitude);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)velocityMagnitude);
        } else if (action == ACTION_STOP) {
            this->actionQueue[0] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::flWheelInt, 0, (float)0);
            this->actionQueue[1] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::frWheelInt, 0, (float)0);
            this->actionQueue[2] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rlWheelInt, 0, (float)0);
            this->actionQueue[3] = new rm3d::ai::ActionLog(rm3d::ai::WheelbotModule::rrWheelInt, 0, (float)0);
        } else if (action == ACTION_SHOOT) {
            //Shoot at the wumpus. The direction is perturbed if noise is turned on, so you may miss. If you hit the wumpus, he dies. If you don't he moves to a new node (randomly)
            PxRaycastHit hit;
            const PxSceneQueryFlags outputFlags = PxSceneQueryFlag::eDISTANCE | PxSceneQueryFlag::eIMPACT | PxSceneQueryFlag::eNORMAL;
            PxVec3 origin = (PxTransform(getCurrentNode())*PxTransform(PxVec3(0,1.0,0))).p;
            PxVec3 direction3 = PxVec3(direction.x, 0, direction.y);
            direction3 += (shootingNoise ? PxVec3(shootResampler(), 0, shootResampler()) : PxVec3(0));
            direction3.normalize();
            bool status = simulator->getSimulationScene()->raycastSingle(origin, direction3, distance, outputFlags, hit);
            RenderLine rl;
            rl.p0 = origin;
            rl.p1 = (PxTransform(origin)*PxTransform(distance*direction3)).p;
            rl.lifeTime = 60;
            renderLines.push_back(rl);
            
            if (status) {
                cout<<"Hit wumpus"<<endl;
                vector<rm3d::ai::Obstacle> obs = simulator->getObstacles();
                for (int i=0; i<obs.size(); i++) {
                    if (strcmp(obs[i].obstacleBody->getName(), hit.shape->getActor().getName()) == 0) {
                        PxRigidDynamic *rd = (PxRigidDynamic *)obs[i].obstacleBody;
                        rd->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
                        rd->addForce(PxVec3(10000, 5000, 0));
                    }
                }
                for (int i=0; i<graphWumpuses.size(); i++) {
                    if (strcmp(graphWumpuses[i].name, hit.shape->getActor().getName()) == 0) {
                        updateGraphOnWumpusDeath(graphWumpuses[i]);
                    }
                }
            } else {
                cout<<"Missed Wumpus"<<endl;
                for (int i=0; i<graphWumpuses.size(); i++) {
                    updateGraphOnWumpusFlee(graphWumpuses[i]);
                }
            }
        }
    }
    
    int getNumGraphNodes() {
        return g.graphPoints.size();
    }
    
    int getNumGraphEdges() {
        return g.graphEdges.size();
    }
    
    /* -------------------------------- END DO NOT MODIFY ------------------------------------------*/
    
    
    
    //Implementation of A* that you may choose to use or not use
    Node* AStar(int startNode,int goalNode, Graph graphObject) {
        priority_queue<Node*, vector<Node*>, CompareNode> open;
        map<int, Node *> openCompare;
        map<int, Node *> closedCompare;
        map<int, double> openFVals;
        vector<Node*> closed;
        vector<vector<int> > Actions(graphObject.graphPoints.size());
        //Compute actions for each state
        for (int i=0; i<graphObject.graphPoints.size(); i++) {
            Actions[i] = vector<int>();
            for (int j=0; j<graphObject.graphEdges.size(); j++) {
                if (graphObject.graphEdges[j].first == i) {
                    Actions[i].push_back(graphObject.graphEdges[j].second);
                }
            }
        }
        
        Node *node = new Node(startNode, NULL, Actions[startNode], (graphObject.graphPoints[startNode] - graphObject.graphPoints[goalNode]).magnitude(), 0);
        open.push(node);
        Node *solution = NULL;
        while (true) {
            if (open.empty()) {
                break;
            }
            Node *n = open.top();
            open.pop();
            if (n->state == goalNode) {
                solution = n;
            }
            for (int i=0; i<n->actions.size(); i++) {
                if(windsNodes[n->actions[i]]){
                    std::cout<<"skip smell Node "<<n->actions[i]<<" for Astart"<<std::endl;
                }else{
                    double h = (graphObject.graphPoints[n->actions[i]] - graphObject.graphPoints[goalNode]).magnitude();
                    double stepCost = 0;
                    stepCost += (graphObject.graphPoints[n->actions[i]] - graphObject.graphPoints[n->state]).magnitude();
                    double g = stepCost + n->gVal;
                    Node *child = new Node (n->actions[i], n, Actions[n->actions[i]], h, g);
                    if (closedCompare[child->state] == NULL && openCompare[child->state] == NULL) {
                        open.push(child);
                        openCompare[child->state] = child;
                        openFVals[child->state] = child->fVal;
                    } else if (openCompare[child->state] != NULL && closedCompare[child->state] == NULL) {
                        if (child->fVal < openFVals[child->state]) {
                            //We must update the current fVal of this node
                            vector<Node *> openNodes;
                            while (!open.empty()) {
                                Node *compNode = open.top();
                                open.pop();
                                if (compNode->state != child->state) {
                                    openNodes.push_back(compNode);
                                } else {
                                    openNodes.push_back(child);
                                }
                            }
                            for (int j=0; j<openNodes.size(); j++) {
                                open.push(openNodes[j]);
                            }
                        }
                    }
            
                }
            }
            closed.push_back(n);
            closedCompare[n->state] = n;
        }
        return solution;
    }
    
    vector<Node *> reconstructSolution(Node *s) {
        vector<Node *> solutionR, solution;
        while (s != NULL)  {
            solutionR.push_back(s);
            s = s->parent;
        }
        for (int i=solutionR.size() - 1; i>=0; i--) {
            solution.push_back(solutionR[i]);
        }
        return solution;
    }
    
    
    void step(void *model) {
        /* -------------------------------- DO NOT MODIFY ----------------------------------------------- */
        simulator->setCustomRenderFunction(render);
        this->model = (rm3d::ai::WheelbotModel *)model;
        currentWheelbotPosition = getCurrentWheelbotPose().p;
        //Housekeeping
        if (this->actionQueue[0] != NULL) delete this->actionQueue[0];
        if (this->actionQueue[1] != NULL) delete this->actionQueue[1];
        if (this->actionQueue[2] != NULL) delete this->actionQueue[2];
        if (this->actionQueue[3] != NULL) delete this->actionQueue[3];
        this->numActions = 4;
        //End Housekeeping
        checkForFailure();
        /* -------------------------------- END DO NOT MODIFY -------------------------------------------- */
        
        //In this project, you can take the action ACTION_SHOOT and a movement action simultaneously
        //When you shoot, the Wumpus hears it and moves randomly. You cannot count on the Wumpus being in the same place throughout your exploration!!!!!
        //For the shoot action, you give it a direction and a distance to shoot. You can shoot in any direction, no matter your orientation
        //When you come across a Wumpus, you will know how far to shoot by reading the magnitude of the smells.
        
        //        PxVec2 direction(shootResampler(), shootResampler());
        ////        direction.normalize();
        ////        if (this->programCounter % 60 == 0)takeAction(ACTION_SHOOT, direction, 5.0);
        //        cout<<"At A Node?: "<<(atANode() ? "True" : "False")<<endl;
        //        vector<PxVec2> actionsForpreviousNode = getActionVectorsForpreviousNode();
        //        vector<float> smells = getCurrentSmells();
        //        vector<float> winds = getCurrentWinds();
        //        for (int i=0; i<actionsForpreviousNode.size(); i++) {
        //            cout<<"Action "<<i<<": ("<<actionsForpreviousNode[i].x<<", "<<actionsForpreviousNode[i].y<<")"<<endl;
        //            cout<<"Wind "<<i<<": "<<winds[i]<<endl;
        //            cout<<"Smell "<<i<<": "<<smells[i]<<endl;
        //        }
        //        if (this->programCounter == 0){
        //            //Learn an initial node
        //            learnedG.graphPoints.push_back(getpreviousNode());
        //            //Learn a RANDOM new node and edge. NOTE: This is FOR ILLUSTRATION PURPOSES ONLY. When you code your solution, you need to learn the nodes and edges of the actual
        //            //map. When you learn a new node, use the getpreviousNode() function to place the node appropriately. You can only learn nodes that you actually visit (or can infer, in
        //            //the case of pits)
        //            learnedG.graphPoints.push_back((PxTransform(getpreviousNode())*PxTransform(PxVec3(5,0,0))).p);
        //            learnedG.graphEdges.push_back(pair<int, int>(1, 0));
        //            cout<<"Adding New Node At: ("<<getpreviousNode().x<<", "<<getpreviousNode().y<<", "<<getpreviousNode().z<<")"<<endl;
        //        }
        //        cout<<"Need to learn a graph with: "<<getNumGraphNodes()<<" nodes"<<endl;
        //        cout<<"and: "<<getNumGraphEdges()<<" edges"<<endl;
        //        cout<<endl<<endl;
        //        cout<<"At A Node?: "<<(atANode() ? "True" : "False")<<endl;
        //if at a ANode save the node and find potential direction to move.
        if(this->programCounter==99999999){
            this->programCounter=1;
        }
        if(this->programCounter==0){
            //initial
            takeAction(ACTION_STOP);
            
            learnedG.graphPoints.push_back(getCurrentNode());
            windsNodes.push_back(false);
            
            //            okG.graphPoints.push_back(getCurrentNode());
            previousNode =getCurrentNode();
            previousPos = 0;
            vector<PxVec2> actionsForCurrentNode = getActionVectorsForCurrentNode();
            vector<float> windsForCurrentNode = getCurrentWinds();
            vector<float> smellsForCurrentNode = getCurrentSmells();
            for(int i=0; i<actionsForCurrentNode.size(); i++) {
                actionsAtNode.push(pair<int,PxVec2>(previousPos,actionsForCurrentNode[i]));
                infoAtNode.push(pair<float,int>(windsForCurrentNode[i],i));
                smellsAtNode.push(smellsForCurrentNode[i]);
                std::cout<<"winds value : "<<windsForCurrentNode[i]<<std::endl;
            }
            if(infoAtNode.top().first!=0){
                std::cout<<"Got Pits: "<<infoAtNode.top().first<<", at node: "<<actionsAtNode.top().first<<std::endl;
                
                PxVec2 actionHere=actionsAtNode.top().second;
                float inteval = infoAtNode.top().first;
                PxVec3 pitNode = PxVec3(previousNode.x+inteval*actionHere.x,previousNode.y,previousNode.z+inteval*actionHere.y);
                learnedG.graphPoints.push_back(pitNode);
                int pitPos =learnedG.graphPoints.size()-1;
                windsNodes.push_back(true);
                learnedG.graphEdges.push_back(pair<int, int>(previousPos, pitPos));
                learnedG.graphEdges.push_back(pair<int, int>(pitPos, previousPos));
                
                infoAtNode.pop();
                actionsAtNode.pop();
            }else{
            
                if(smellsForCurrentNode[infoAtNode.top().second]!=0){
                    std::cout<<"Smelly Shoot!"<<std::endl;
                    takeAction(ACTION_SHOOT, actionsAtNode.top().second, infoAtNode.top().second);
                }
                currentAction = actionsAtNode.top().second;
                actionsAtNode.pop();
                infoAtNode.pop();
            }
        }
        //        if(actionsAtNode.empty()){
        //            takeAction(ACTION_STOP);
        //            std::cout<<"DONE"<<std::endl;
        //        }
        PxVec2 pos2D = PxVec2(getCurrentWheelbotPose().p.x, getCurrentWheelbotPose().p.z);
        PxVec3 posAlongX = (getCurrentWheelbotPose()*PxTransform(PxVec3(1,0,0))).p;
        PxVec2 posAlongX2D = PxVec2(posAlongX.x, posAlongX.z);
        PxVec2 goal2D;
        if(atANode()&&getCurrentNode()!=previousNode){
            //                std::cout<<"Found Adjustment to Node"<<std::endl;
            goal2D = PxVec2(getCurrentNode().x, getCurrentNode().z);
            currentAction=goal2D - pos2D;
        }
//        if(Shoot){
//            std::cout<<"Smelly Shoot!"<<std::endl;
//            takeAction(ACTION_SHOOT, currentAction, shootDistance);
//        }
        PxVec2 A = posAlongX2D - pos2D;
        
        PxVec2 B =currentAction;
        
        A.normalize();
        B.normalize();
        
        float thetaA = atan2(A.x, A.y);
        float thetaB = atan2(B.x, B.y);
        
        float thetaAB = thetaB - thetaA;
        
        while (thetaAB <= - PxPi)
            thetaAB += 0.1 * PxPi;
        
        while (thetaAB > PxPi)
            thetaAB -= 0.1 * PxPi;
        
        //Here is the result: the angular error between Wheelbot's heading and the goal direction
        PxReal angleError = thetaAB;
        
        //If the angular error is too large
        if (fabs(angleError) > 0.005) {
            //turn right if it is less than zero, otherwise turn left
            if (angleError < 0) {
                takeAction(ACTION_RIGHT);
            } else {
                takeAction(ACTION_LEFT);
            }
            //If we are close enough to the goal
        } else if (atANode()&& getCurrentNode()!=previousNode&&(goal2D - pos2D).magnitude() <= 0.1) {
            //stop
            takeAction(ACTION_STOP);
            //save current smell info
            vector<float> smellsForCurrentNode = getCurrentSmells();

            Shoot = false;
            
            PxVec3 currentNode =getCurrentNode();
            //learn new Node
            int currentPos = -1;
            
            for(int i=0;i<learnedG.graphPoints.size();i++){
                if(learnedG.graphPoints[i]==currentNode){
                    currentPos=i;
                    break;
                }
            }
            if(currentPos<0){
                std::cout<<"find a new point";
                
                learnedG.graphPoints.push_back(currentNode);
                windsNodes.push_back(false);
                //                    okG.graphPoints.push_back(currentNode);
                currentPos =learnedG.graphPoints.size()-1;
                vector<PxVec2> actionsForCurrentNode = getActionVectorsForCurrentNode();
                vector<float> windsForCurrentNode = getCurrentWinds();
                for(int i=0; i<actionsForCurrentNode.size(); i++) {
                    actionsAtNode.push(pair<int,PxVec2>(currentPos,actionsForCurrentNode[i]));
                    infoAtNode.push(pair< float,int >(windsForCurrentNode[i],i));
//                    smellsAtNode.push(smellsForCurrentNode[i]);
                }
                
                
            }
            if(solution.empty()){
                
                
                
                std::cout<<"Current Position "<<currentPos<<std::endl;
                
                
                if((std::find(learnedG.graphEdges.begin(), learnedG.graphEdges.end(),pair<int, int>(previousPos, currentPos))==learnedG.graphEdges.end())){
                    learnedG.graphEdges.push_back(pair<int, int>(previousPos, currentPos));
                    learnedG.graphEdges.push_back(pair<int, int>(currentPos, previousPos));
                    //                        okG.graphEdges.push_back(pair<int, int>(previousPos, currentPos));
                    //                        okG.graphEdges.push_back(pair<int, int>(currentPos, previousPos));
                    std::cout<<"new place"<<currentPos<<std::endl;
                    
                }
                if(actionsAtNode.empty()){
                    takeAction(ACTION_STOP);
                    
                    std::cout<<"Map explored fully, found edges:"<<learnedG.graphEdges.size()<<", point:"<<learnedG.graphPoints.size()<<std::endl;
                    std::cout<<"             Original Map,edges:"<<g.graphEdges.size()<<", point:"<<g.graphPoints.size()<<std::endl;
                    
                    takeAction(ACTION_STOP);
                    
                    
                    //                    }else if(infoAtNode.top().first!=0){
                    //                        while(infoAtNode.top().first!=0){
                    //                            std::cout<<"Got Pits: "<<infoAtNode.top().first<<", at node: "<<actionsAtNode.top().first<<std::endl;
                    //
                    //                            PxVec2 actionHere=actionsAtNode.top().second;
                    //                            float inteval = infoAtNode.top().first;
                    //                            PxVec3 pitNode = PxVec3(currentNode.x+inteval*actionHere.x,currentNode.y,currentNode.z+inteval*actionHere.y);
                    //                            int pitPos = learnedG.graphPoints.size()-1;;
                    //
                    //                            for(int i=0;i<learnedG.graphPoints.size();i++){
                    //                                if(learnedG.graphPoints[i]==pitNode){
                    //                                    pitPos=i;
                    //                                    break;
                    //                                }
                    //                            }
                    //
                    //                            learnedG.graphPoints.push_back(pitNode);
                    //                            learnedG.graphEdges.push_back(pair<int, int>(currentPos, pitPos));
                    //                            learnedG.graphEdges.push_back(pair<int, int>(pitPos, currentPos));
                    //
                    //
                    //                            infoAtNode.pop();
                    //                            actionsAtNode.pop();
                    //                        }
                    
                }else{
                    if(currentPos != actionsAtNode.top().first){
                        //no more route get back to previous node
                        //                            wantBack=true;
                        solution =reconstructSolution(AStar(currentPos,actionsAtNode.top().first,learnedG));
                        previousNode =   currentNode;
                        previousPos  =   currentPos;
                        solution.erase(solution.begin());
                        
                        currentAction = getVectorForActionAtNode(solution.front()->state,-1);
                        std::cout<<"no action get back to "<<actionsAtNode.top().first<<", first step "<<solution.front()->state<<std::endl;
                        
                        solution.erase(solution.begin());
                    }else if(infoAtNode.top().first!=0){
                        //                            while(infoAtNode.top().first!=0){
                        std::cout<<"Got Pits: "<<infoAtNode.top().first<<", at node: "<<actionsAtNode.top().first<<std::endl;
                        
                        PxVec2 actionHere=actionsAtNode.top().second;
                        float inteval = infoAtNode.top().first;
                        PxVec3 pitNode = PxVec3(currentNode.x+inteval*actionHere.x,currentNode.y,currentNode.z+inteval*actionHere.y);
                        int pitPos = -1;
                        
                        for(int i=0;i<learnedG.graphPoints.size();i++){
                            if((learnedG.graphPoints[i]-pitNode).magnitude()<=1.0){
                                pitPos=i;
                                break;
                            }
                        }
                        
                        if(pitPos<0){
                            learnedG.graphPoints.push_back(pitNode);
                            pitPos = learnedG.graphPoints.size()-1;
                            windsNodes.push_back(true);
                            
                        }
                        
                        learnedG.graphEdges.push_back(pair<int, int>(currentPos, pitPos));
                        learnedG.graphEdges.push_back(pair<int, int>(pitPos, currentPos));
                        

                        infoAtNode.pop();
                        actionsAtNode.pop();
                        //                            }
                        
                    }else{
                        std::cout<<"new route at"<<currentPos<<", wind value:"<<infoAtNode.top().first<<", Wompus value"<<infoAtNode.top().second<<std::endl;
                        //                            wantBack =false;
                        
                        previousNode =   currentNode;
                        previousPos  =   currentPos;
                        std::cout<<"("<<actionsAtNode.top().second.x<<","<<actionsAtNode.top().second.y<<")"<<std::endl;
                        
                        if(smellsForCurrentNode[infoAtNode.top().second]!=0){
                            std::cout<<"Smelly Shoot!"<<std::endl;
                            takeAction(ACTION_SHOOT, actionsAtNode.top().second, smellsForCurrentNode[infoAtNode.top().second]);
                        }
                        currentAction = actionsAtNode.top().second;
                        actionsAtNode.pop();
                        infoAtNode.pop();
                        
                    }
                }
                
            }else{
                previousNode =   currentNode;
                previousPos  =   currentPos;
                currentAction = getVectorForActionAtNode(solution.front()->state,-1);
                std::cout<<"Astart back to "<<solution.front()->state<<std::endl;
                
                solution.erase(solution.begin());
            }
            
        }else {
            takeAction(ACTION_FORWARD);
        }
        
        
        
        
        this->programCounter++;
    }
};

//Current position of Wheelbot
PxVec3 Project3_Control::currentWheelbotPosition;
//A plan of points to visit
vector<int> Project3_Control::plan;
//Radius around goal specifying when we have arrived at a goal
PxReal Project3_Control::currentGoalRadius = .30;
//Graph object on which to plan
Graph Project3_Control::g;
int Project3_Control::goal;
int Project3_Control::start;
Graph Project3_Control::learnedG;
vector<vector<int> > Project3_Control::actions;
vector<int> Project3_Control::graphPits;
float Project3_Control::minDistance = 5.0;
bool Project3_Control::shootingNoise = true;
vector<float> Project3_Control::graphWinds;
vector<float> Project3_Control::graphSmell;
vector<Wumpus> Project3_Control::graphWumpuses;
boost::mt19937 Project3_Control::rngShoot;
boost::normal_distribution<> Project3_Control::shootDistribution(0.0, 10.0);
boost::variate_generator< boost::mt19937, boost::normal_distribution<> > Project3_Control::shootResampler(rngShoot, shootDistribution);
vector<RenderLine> Project3_Control::renderLines;
//Destk atkl
#endif

