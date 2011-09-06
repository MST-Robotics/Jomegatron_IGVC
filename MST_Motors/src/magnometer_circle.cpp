#include <signal.h>
#include "DriveTank.h"

#define SPEED 1.00
#define TURN 0.065616798

bool quit;
void quit_function(int signal)
{
	quit=true;
	return;
}

int main()
{
	quit = false;
	signal( SIGINT, quit_function );
	DriveTank Tank;
	double turnRate = 0.065616798;
	double forwardRate = 1.0;
	int shutdown = 100;
	
	Tank.SetVelocity( forwardRate, turnRate );
	
	while( shutdown > 0 && !quit )
	{
		Tank.kickTheDog = true;
		sleep(1);
		shutdown--;
	}
	
	Tank.SetVelocity(0.0,0.0);
	
	return 0;	
}
