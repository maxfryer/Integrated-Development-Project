#ifndef ROBOT
#define ROBOT
#include  "RobotFunctions.cpp"
#endif

class TaskManager: public Robot{
    public:

    enum class Tasks {FIND_BOX,DEAL_WITH_BLUE_BOX,DEAL_WITH_RED_BOX};
    Tasks Task;

    void runTasks(){

        switch(Task){

            case Tasks::DEAL_WITH_BLUE_BOX:
                break;
            case Tasks::DEAL_WITH_RED_BOX:
                break;
            case Tasks::FIND_BOX:
                //will run once every loop HAS to be none blocking except when left right 180 turning etc
                break;
        }
    }
};


