/*
 * Written by Mathias Thor DEC 30
 * Happy NewYear!
 */

#include "neutronController.h"
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

std::atomic<bool> quit(false);    // signal flag

void got_signal(int)
{
    //system("blinkstick --morph --set-color green --brightness 40");

    quit.store(true);
}

int main(int argc,char* argv[])
{
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);
    //system("blinkstick --morph --set-color red --brightness 40");

    neutronController * controller;
    controller = new neutronController(argc,argv);

    int controllerStatus = 1;
    while(controllerStatus == 1 || controllerStatus == 2){
        controllerStatus = controller->runController();
        if(controllerStatus == 2){
            //cout << "[ INFO] Restarting in main..." << endl;
            delete controller;
            controller = new neutronController(argc,argv);
            controllerStatus = 1;
        }
        if( quit.load() ) break;
    }

    delete controller;

    return(0);
}


