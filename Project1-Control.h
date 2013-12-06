

#ifndef ReMod3D_Project1_Control_h
#define ReMod3D_Project1_Control_h

/*
 * Keyboard Map
 * See Camera.h for detailed camera control information
 * Buttons 1,2,3,4,5,6,w,a,s,d,(up arrow),(down arrow),(left arrow),(right arrow) control the camera
 * Holding down the left mouse button and moving the mouse manipulates the orientation of the camera
 * Button p sets program pause state (pause/unpause)
 * Button f switches between Wheelbot view and global view (NOTE: Color blob sensing will not work in global view, and the camera cannot be manually
 * moved in Wheelbot view)
 */


///DO NOT MODIFY!!!!------------------------///

#include "ModuleProgram.h"
#include "WheelbotModule.h"
#include "SimulationUtility.h"
#include "ActionLog.h"

//Possible Wheelbot actions
enum WheelbotActions {
    ACTION_FORWARD = 0,
    ACTION_BACKWARD,
    ACTION_LEFT,
    ACTION_RIGHT,
    ACTION_STOP
};

//Position of the current goal
static PxVec3 currentGoalPosition;
//Radius of hemisphere surrounding current goal. If Wheelbot enters this hemisphere, it has reached the goal
static const PxReal currentGoalRadius = 0.4;
//Magnitude of the velocity of the wheels. You should not need to tune this unless wheelbot's wheels are too fast and cause flipping
static const PxReal velocityMagnitude = 10.0;
//Upper and lower bounds for stochastically choosing goal position/radius
static const PxReal lowerBoundGoalX = 0.0;
static const PxReal lowerBoundGoalY = 0.0;
static const PxReal upperBoundGoalX = 5.0;
static const PxReal upperBoundGoalY = 5.0;
static const PxReal areaTolerance = 20000.0;
//Possible goal colors
static vector<rm3d::Color> colors;
//Obstacle material
static PxMaterial *obMaterial;


class Project1_Control:rm3d::ai::ModuleProgram<rm3d::ai::WheelbotDock *> {
private:
    WheelbotSimBase* simulator;
    rm3d::ai::WheelbotModel *model;
public:
    
    void setNewGoal() {
        //Choose new goal position randomly
        currentGoalPosition = PxVec3(rm3d::SimulationUtility::randomFloatInRange(lowerBoundGoalX, upperBoundGoalX)*((rand() % 2 == 1) ? -1: 1),
                                     0,
                                     rm3d::SimulationUtility::randomFloatInRange(lowerBoundGoalY, upperBoundGoalY))*((rand() % 2 == 1) ? -1: 1);
        simulator->clearObstacles();
        //Create a new obstacle that is red, green, or yellow.
        simulator->createObstacle(PxTransform(currentGoalPosition), PxSphereGeometry(0.3/2.0), obMaterial, 1000, 1000, "ob",
                                  true, false, colors[rand()%3]);
    }
    
    Project1_Control(WheelbotSimBase* simulator) {
        colors.push_back(rm3d::Color(0,1,0));
        colors.push_back(rm3d::Color(1,0,0));
        colors.push_back(rm3d::Color(1,1,0));
        obMaterial = simulator->getSimulationPhysics()->createMaterial(0.5, 0.5, 0.5);
        this->programCounter = 0;
        this->actionQueue = new rm3d::ai::ActionLog*[4];
        this->simulator = simulator;
        this->actionQueue[0] = NULL;
        this->actionQueue[1] = NULL;
        this->actionQueue[2] = NULL;
        this->actionQueue[3] = NULL;
        setNewGoal();
    }
    
    ~Project1_Control() {
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
    
    PxTransform getCurrentWheelbotPose() {
        return boost::any_cast<PxTransform>(this->model->getCurrentState()->sensorValues[rm3d::ai::WheelbotModule::posOrientInt]);
    }
    
    PxVec3 getGoalPosition() {
        return currentGoalPosition;
    }
    
    vector<rm3d::Blob> getCurrentColorBlobs() {
        vector<rm3d::Blob> localBlobs;
        vector<rm3d::Blob> blobs = boost::any_cast<vector<rm3d::Blob> >(this->model->getCurrentState()->sensorValues[rm3d::ai::WheelbotModule::colorBlobInt]);
        for (int i=0; i<blobs.size(); i++) {
            if (blobs[i].getColor() == rm3d::BlobBlue || blobs[i].getColor() == rm3d::BlobBrown) {
                localBlobs.push_back(blobs[i]);
            } else {
                if ((blobs[i].getColor() == rm3d::BlobRed || blobs[i].getColor() == rm3d::BlobYellow || blobs[i].getColor() == rm3d::BlobGreen) &&
                    (getCurrentWheelbotPose().p - getGoalPosition()).magnitude() < 0.8) {
                    localBlobs.push_back(blobs[i]);
                }
            }
        }
        return localBlobs;
    }
    
    void takeAction(WheelbotActions action) {
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
        }
    }
    
    ///END DO NOT MODIFY------------------///
    
    
    void step(void *model) {
        ///DO NOT MODIFY------------------///
        
        this->model = (rm3d::ai::WheelbotModel *)model;
        //Housekeeping
        if (this->actionQueue[0] != NULL) delete this->actionQueue[0];
        if (this->actionQueue[1] != NULL) delete this->actionQueue[1];
        if (this->actionQueue[2] != NULL) delete this->actionQueue[2];
        if (this->actionQueue[3] != NULL) delete this->actionQueue[3];
        //End Housekeeping
        
        ///END DO NOT MODIFY------------------///
        
        
        
        ///BEGIN TODO ------------------------------------///
        ////Available Sensors:
        //1. Wheelbot pose sensor: returns the position and orientation of Wheelbot (as a PxTransform). To use this sensor, call getCurrentWheelbotPose()
        //2. Goal position sensor: returns the 3D point representing the location of the goal (as a PxVec3). To use this sensor, call getGoalPosition()
        //3. Color blob sensor: returns the blobs of color (as a vector of rm3d::Blob) picked up by Wheelbot's camera. To use this sensor, call getCurrentColorBlobs()
        ///Available Actions:
        //1. Forward: Move Wheelbot forward. To perform this action call, takeAction(ACTION_FORWARD)
        //2. Backward: Move Wheelbot backward. To perform this action call, takeAction(ACTION_BACKWARD)
        //3. Left: Move Wheelbot left. To perform this action call, takeAction(ACTION_LEFT)
        //4. Right: Move Wheelbot right. To perform this action call, takeAction(ACTION_RIGHT)
        //5. Stop: Stop Wheelbot. To perform this action call, takeAction(ACTION_STOP)
        
        ///FILL IN THIS PART OF THE CODE - THE CODE BELOW IS SIMPLY A PLACEHOLDER ILLUSTRATING HOW TO EXECUTE ACTIONS AND READ SENSORS
        //1. Determine how to reach the goal using the given sensors and actions
        //2. Stop when you have reached the goal within a tolerance of currentGoalRadius, a constant defined above
        //3. Print the color of the goal object to the screen.
        
        
        
        PxVec3 goal = getGoalPosition();
        PxTransform pose = getCurrentWheelbotPose();
        PxVec3 position = pose.p;
        PxVec3 goalInWheelbotFrame = getCurrentWheelbotPose().getInverse().transform(goal);
        if((int)(goalInWheelbotFrame.z*100)<0){
            takeAction(ACTION_LEFT);
            cout<<"Adjust the direction. Turn Left."<<endl;

        }else if((int)(goalInWheelbotFrame.z*100)>0){
            takeAction(ACTION_RIGHT);
            cout<<"Adjust the direction. Turn Right."<<endl;

        }else{
            if((goalInWheelbotFrame.x*goalInWheelbotFrame.x+goalInWheelbotFrame.z*goalInWheelbotFrame.z)<currentGoalRadius*currentGoalRadius){
                takeAction(ACTION_STOP);
                vector<rm3d::Blob> blobs = getCurrentColorBlobs();
                for(int i=0;i<blobs.size();i++){
                    if (blobs[i].getColor() ==rm3d::BlobGreen) {
                        cout<<"Reach the Goal! Color is Green!!!"<<endl;
                    }else if (blobs[i].getColor() ==rm3d::BlobYellow) {
                        cout<<"Reach the Goal! Color is Yellow!!!"<<endl;
                        
                    }else if (blobs[i].getColor() ==rm3d::BlobRed) {
                        cout<<"Reach the Goal! Color is Red!!!"<<endl;
                        
                    }
                }
            }else{
                cout<<"(goal to wheel"<<goalInWheelbotFrame.x<<","<<goalInWheelbotFrame.y<<","<<goalInWheelbotFrame.z<<")"<<endl ;
                takeAction(ACTION_FORWARD);
            }
        }
        
        
        ///END TODO ----------------------------------////
        this->programCounter++;
    }
};
#endif
