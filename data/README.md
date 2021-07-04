# Data format
The following explains the files located within this directory and its sub-directories.

[![Sparkline](https://stars.medv.io/Naereen/badges.svg)](https://stars.medv.io/Naereen/badges)

## Root/this directory
Contains the nine weight sets in `.json` format for the base controller and the eight control modules. It also contains log files with the activity of the behaviors for each leg as well as sensory information for the primitive behaviors (in `.txt` files).

## Jobs
Contains the data from an ongoing learning run _(or job)_. The `RL_job.json` files contain parameter and noise sets (i.e., policies) for the simulation workers. The _answers directory_ contains answers _(i.e., the final rewards)_ from the simulation workers when they finish a job. Finally, `RL_log.txt` contains information about the minimum, mean, and maximum reward and sub-rewards for each iteration.

## Storage directory
Contains the data collected from an entire learning run _(or job)_ when using `RL_repeater.sh`. I.e., it will contain the `RL_job.json` and `RL_log.txt` files. Currently is contains the data from learning the base controller and the eight control modules _(each repeated five times)_.

[![GPLv3 license](https://img.shields.io/badge/License-GPLv3-blue.svg)](http://perso.crans.org/besson/LICENSE.html)
