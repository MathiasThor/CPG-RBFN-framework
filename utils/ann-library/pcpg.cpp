#include "pcpg.h"

PCPG::PCPG():set(2),countup(2),countdown(2)
{
	/*******************************************************************************
	*  MODULE 2 CPG POST-PROCESSING
	*******************************************************************************/
				pcpg_step.resize(2);
				setold.resize(2);
				countupold.resize(2);
				countdownold.resize(2);
				diffset.resize(2);
				deltaxdown.resize(2);
				deltaxup.resize(2);
				xup.resize(2);
				xdown.resize(2);

				yup.resize(2);
				ydown.resize(2);
				pcpg_output.resize(2);

	 setNeuronNumber(2);

}
void PCPG::updateOutputs(){
			//***CPG post processing*****

			pcpg_step.at(0) = getActivity(0);
			pcpg_step.at(1) = getActivity(1);

			setold.at(0) = set.at(0);
			setold.at(1) = set.at(1);


			countupold.at(0) = countup.at(0);
			countupold.at(1) = countup.at(1);


			countdownold.at(0) = countdown.at(0);
			countdownold.at(1) = countdown.at(1);

			//1) Linear threshold transfer function neuron 1 , or called step function neuron//
			/********************************************************/

			if (pcpg_step.at(0)>=0.85) ////////////////////Intuitively select
			{
				set.at(0) = 1.0;
			}
			if (pcpg_step.at(0)<0.85) ////////////////////Intuitively select
			{
				set.at(0) = -1.0;
			}


			if (pcpg_step.at(1)>=0.85) ////////////////////Intuitively select
			{
				set.at(1) = 1.0;
			}
			if (pcpg_step.at(1)<0.85) ////////////////////Intuitively select
			{
				set.at(1) = -1.0;
			}


			diffset.at(0) = set.at(0)-setold.at(0); // double
			diffset.at(1) = set.at(1)-setold.at(1); // double


			//2) Count how many step of Swing
			/********************************************************/

			if (set.at(0) == 1.0)
			{
			countup.at(0) = countup.at(0)+1.0; //Delta x0 up
			countdown.at(0) = 0.0;
			}

			// Count how many step of Stance
			else if (set.at(0) == -1.0)
			{
			countdown.at(0) = countdown.at(0)+1.0; //Delta x0 down
			countup.at(0) = 0.0;
			}


			if (set.at(1) == 1.0)
			{
			countup.at(1) = countup.at(1)+1.0; //Delta x0 up
			countdown.at(1) = 0.0;
			}

			// Count how many step of Stance
			else if (set.at(1) == -1.0)
			{
			countdown.at(1) = countdown.at(1)+1.0; //Delta x0 down
			countup.at(1) = 0.0;
			}

			//3) Memorized the total steps of swing and stance
			/********************************************************/



			if (countup.at(0) == 0.0 && diffset.at(0) == -2.0 && set.at(0) == -1.0)
			deltaxup.at(0) = countupold.at(0);
	//
			if (countdown.at(0) == 0.0 && diffset.at(0) == 2.0 && set.at(0) == 1.0)
			deltaxdown.at(0) = countdownold.at(0);
	//
	//
			if (countup.at(1) == 0.0 && diffset.at(1) == -2.0 && set.at(1) == -1.0)
			deltaxup.at(1) = countupold.at(1);
	//
			if (countdown.at(1) == 0.0 && diffset.at(1) == 2.0 && set.at(1) == 1.0)
			deltaxdown.at(1) = countdownold.at(1);

			//4) Comput y up and down !!!!
			/********************************************************/

			xup.at(0) =  countup.at(0);
			xdown.at(0) = countdown.at(0);

			xup.at(1) =  countup.at(1);
			xdown.at(1) = countdown.at(1);



			////////////Scaling Slope Up calculation////////
			yup.at(0) = ((2./deltaxup.at(0))*xup.at(0))-1;
			////////////Scaling  Slope Down calculation//////
			ydown.at(0) = ((-2./deltaxdown.at(0))*xdown.at(0))+1;


			////////////Scaling Slope Up calculation////////
			yup.at(1) = ((2./deltaxup.at(1))*xup.at(1))-1;
			////////////Scaling  Slope Down calculation//////
			ydown.at(1) = ((-2./deltaxdown.at(1))*xdown.at(1))+1;


			//5) Combine y up and down !!!!
			/********************************************************/

			if (set.at(0) >= 0.0)
			pcpg_output.at(0) = yup.at(0);

			if (set.at(0) < 0.0)
			pcpg_output.at(0) = ydown.at(0);


			if (set.at(1) >= 0.0)
			pcpg_output.at(1) = yup.at(1);

			if (set.at(1) < 0.0)
			pcpg_output.at(1) = ydown.at(1);


			//********Limit upper and lower boundary

			if(pcpg_output.at(0)>1.0)
			pcpg_output.at(0) = 1.0;
			else if(pcpg_output.at(0)<-1.0)
			pcpg_output.at(0) = -1.0;

			if(pcpg_output.at(1)>1.0)
			pcpg_output.at(1) = 1.0;
			else if(pcpg_output.at(1)<-1.0)
			pcpg_output.at(1) = -1.0;

			//***CPG post processing*end*

			setOutput(0,pcpg_output.at(0));
			setOutput(1,pcpg_output.at(1));


}
