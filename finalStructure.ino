
//HORRENDOUS COS ALL FUNCTIONS NEED TO BE WITHOUT WHILE LOOPS WHERE POSS

#include "RobotFunctions.ino"

class TaskManager: public Robot{
    public:

    void runTasks(){


        switch tasks

        case findBox(){

            //will run once every loop HAS to be none blocking except when left right 180 turning etc

            navigateToTjunctionFromPosition();
            Robot::turnLeft();
            findBoxAndStoreColour();
            if(box is blue) place first blue; //so switch task to palcing the blue until that is completed 
            if (box is red) dealWithReds; //so switch tasks to placing red until that 
        
       
        case dealWithBlue(){
            //do task
            change task to find box again
        }

        case dealWithReds(){

        }
    }


};


TaskManager Bot;

void loop(){
    if(run == true){
        Bot.checkAllSensorValues(false);
        Bot.flashLEDS();
        Bot.runTasks();
    }
}


