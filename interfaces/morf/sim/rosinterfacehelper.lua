-----
----- Created by mat.
----- DateTime: 5/22/19 1:14 PM
-----
--
--function sysCall_init()
--
--    -- EXTRACT SIM NUMBER
--    file = "remoteApiConnections.txt"
--    local f = assert(io.open(file, "rb"))
--
--    lines = {}
--    for line in io.lines(file) do
--        lines[#lines + 1] = line
--    end
--
--    local array = {}
--    for i in string.gmatch(lines[12], "%d+") do
--        table.insert(array, i)
--    end
--
--    nameAddOn = tostring(19998-tonumber(array[2]))
--    print("Simulation Number: " .. nameAddOn)
--
--    if simROS then
--        print("<font color='#0F0'>ROS interface was found.</font>@html")
--        enableSynModeSub=simROS.subscribe('/sim_control'..nameAddOn..'/enableSyncMode', 'std_msgs/Bool', 'enableSyncMode_callback')
--        triggerNextStepSub=simROS.subscribe('/sim_control'..nameAddOn..'/triggerNextStep', 'std_msgs/Bool', 'triggerNextStep_callback')
--
--        simStepDonePub=simROS.advertise('/sim_control'..nameAddOn..'/simulationStepDone', 'std_msgs/Bool')
--        simStatePub=simROS.advertise('/sim_control'..nameAddOn..'/simulationState','std_msgs/Int32')
--        simTimePub=simROS.advertise('/sim_control'..nameAddOn..'/simulationTime','std_msgs/Float32')
--        auxPub=simROS.advertise('/sim_control'..nameAddOn..'/privateMsgAux', 'std_msgs/Bool')
--        auxSub=simROS.subscribe('/sim_control'..nameAddOn..'/privateMsgAux', 'std_msgs/Bool', 'aux_callback')
--
--        rosInterfaceSynModeEnabled=true
--        haltMainScript=false
--    else
--        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
--    end
--end
--
--function enableSyncMode_callback(msg)
--    rosInterfaceSynModeEnabled=msg.data
--    haltMainScript=rosInterfaceSynModeEnabled
--end
--
--function triggerNextStep_callback(msg)
--    haltMainScript=false
--    simROS.publish(simStepDonePub,{data=false})
--end
--
--function aux_callback(msg)
--    simROS.publish(simStepDonePub,{data=true})
--end
--
--function publishSimState()
--    local state=0 -- simulation not running
--    local s=sim.getSimulationState()
--    if s==sim.simulation_paused then
--        state=2 -- simulation paused
--    elseif s==sim.simulation_stopped then
--        state=0 -- simulation stopped
--    else
--        state=1 -- simulation running
--    end
--    simROS.publish(simStatePub,{data=state})
--end
--
--function sysCall_nonSimulation()
--    if simROS then
--        publishSimState()
--    end
--end
--
--function sysCall_beforeMainScript()
--    return {doNotRunMainScript=haltMainScript}
--end
--
--function sysCall_actuation()
--    if simROS then
--        publishSimState()
--        simROS.publish(simTimePub,{data=sim.getSimulationTime()})
--    end
--end
--
--function sysCall_sensing()
--    if simROS then
--        simROS.publish(auxPub,{data=true})
--        haltMainScript=rosInterfaceSynModeEnabled
--    end
--end
--
--function sysCall_suspended()
--    if simROS then
--        publishSimState()
--    end
--end
--
--function sysCall_afterSimulation()
--    if simROS then
--        publishSimState()
--    end
--end
--
--function sysCall_cleanup()
--    if simROS then
--        simROS.shutdownSubscriber(enableSynModeSub)
--        simROS.shutdownSubscriber(triggerNextStepSub)
--        simROS.shutdownSubscriber(auxSub)
--        simROS.shutdownPublisher(auxPub)
--        simROS.shutdownPublisher(simStepDonePub)
--        simROS.shutdownPublisher(simStatePub)
--        simROS.shutdownPublisher(simTimePub)
--    end
--end