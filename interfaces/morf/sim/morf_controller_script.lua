--[[
Callback function for receiving motor positions over ROS
--]]

-- Constants
math.randomseed(os.time())
math.random(); math.random(); math.random()
local utils = dofile "/home/mat/workspace/gorobots/projects/C-CPGRBFN/CPGRBFN_feedback_v3/interfaces/morf/sim/utils.lua" -- TODO Fix path

function ballTracker( )
    posMorf = sim.getObjectPosition(frontMorf, -1)
    posNxtBall = sim.getObjectPosition(ball, -1)
    distanceToNxtBall = math.sqrt(math.pow((posNxtBall[1] - posMorf[1]),2) + math.pow((posNxtBall[2] - posMorf[2]),2))

    --print(distanceToNxtBall)

    if nxtBall == 0 then
        result=sim.setShapeColor(ball, nil, 0, {0.30, 0.94, 0.00})
    end

    if distanceToNxtBall < 0.2 and nxtBall < maxNxtBall then
        -- nxtBall change color to red:
        result=sim.setShapeColor(ball, nil, 0, {1.00, 0.00, 0.00})

        --sim.removeObject(ball)
        ball = sim.getObjectHandle("ball"..nxtBall)
        nxtBall = nxtBall + 1

        -- nxtBall change color to green:
        result=sim.setShapeColor(ball, nil, 0, {0.30, 0.94, 0.00})

        print("next ball: " .. nxtBall)

        if primitive ~= true then
          offset = 2
          if nxtBall == 1+offset
              then -- NARROW
              behavior_signal[3] = 1
              print("Narrow behavior activated")
          elseif nxtBall == 2+offset
              then -- WALL
              behavior_signal[3] = 0
              behavior_signal[5] = 1
              print("Wall behavior activated")
          elseif nxtBall == 3+offset
              then -- NARROW
              behavior_signal[3] = 1
              behavior_signal[5] = 0
              print("Narrow behavior activated")
          elseif nxtBall == 4+offset
              then -- HIGH
              behavior_signal[1] = 1
              behavior_signal[3] = 0
              print("High behavior activated")
          elseif nxtBall == 5+offset
              then -- NO SKILL
              behavior_signal[1] = 0
          elseif nxtBall == 6+offset
              then -- NO SKILL
              behavior_signal[1] = 0
          elseif nxtBall == 7+offset
              then -- WALK NO MED
          elseif nxtBall == 8+offset
              then -- WALK NO MED
  --             behavior_signal[6] = 1
  --             kill_middle_legs = true
  --             print("WalkNoMed behavior activated")
          elseif nxtBall == 9+offset
              then -- NO SKILL
  --             behavior_signal[6] = 0
  --             kill_middle_legs = false
          elseif nxtBall == 10+offset
              then -- PIPE
              behavior_signal[4] = 1
              print("Pipe behavior activated")
          elseif nxtBall == 11+offset
              then -- NO SKILL
              behavior_signal[4] = 0
              behavior_signal[1] = 1
          elseif nxtBall == 12+offset
              then -- NO SKILL
              behavior_signal[1] = 0
          elseif nxtBall == 13+offset
              then -- LOW
              behavior_signal[2] = 1
              print("Low behavior activated")
          elseif nxtBall == 14+offset
              then -- no skill
              behavior_signal[2] = 0
          end
      end
    end

    -- ball1 = narrow
    -- ball2 = wall
    -- ball3 = narrow
    -- ball4 = no skill
    -- ball5 = high
    -- ball6 = no skill
    -- ball7 = high
    -- ball8 = pipe
    -- ball9 = high
    -- ball10 = no skill
    -- ball11 = low
    -- ball12 = no skill
end

function setMotorPositions_cb(msg)
    data = msg.data

    if jointPos_noise then
        data = utils.noise(data, "gaussian", {0, 0.0001})
    end

-- WHAT WE USE:
    sim.setJointTargetPosition(TC_motor0, jointLimiter(data[2], -110, 22) )
    sim.setJointTargetPosition(CF_motor0, jointLimiter(data[4], -60, 130) )
    sim.setJointTargetPosition(FT_motor0, jointLimiter(data[6], -175, 0) )

    sim.setJointTargetPosition(TC_motor1, jointLimiter(data[8], -22, 22) )
    sim.setJointTargetPosition(CF_motor1, jointLimiter(data[10], -60, 130) )
    sim.setJointTargetPosition(FT_motor1, jointLimiter(data[12], -175, 0) )

    sim.setJointTargetPosition(TC_motor2, jointLimiter(data[14], -22, 110) )
    sim.setJointTargetPosition(CF_motor2, jointLimiter(data[16], -60, 130) )
    sim.setJointTargetPosition(FT_motor2, jointLimiter(data[18], -175, 0) )

    sim.setJointTargetPosition(TC_motor3, jointLimiter(data[20], -22, 110) )
    sim.setJointTargetPosition(CF_motor3, jointLimiter(data[22], -60, 130) )
    sim.setJointTargetPosition(FT_motor3, jointLimiter(data[24], -175, 0) )

    sim.setJointTargetPosition(TC_motor4, jointLimiter(data[26], -22, 22) )
    sim.setJointTargetPosition(CF_motor4, jointLimiter(data[28], -60, 130) )
    sim.setJointTargetPosition(FT_motor4, jointLimiter(data[30], -175, 0) )

    sim.setJointTargetPosition(TC_motor5, jointLimiter(data[32], -110, 22) )
    sim.setJointTargetPosition(CF_motor5, jointLimiter(data[34], -60, 130) )
    sim.setJointTargetPosition(FT_motor5, jointLimiter(data[36], -175, 0) )
end

--[[
Callback function for displaying data on a graph
--]]
function graph_cb(msg)
    data = msg.data
    if simGetSimulationTime() > 1 then
        simSetGraphUserData(graphHandle,"command",data[1])
        simSetGraphUserData(graphHandle,"actual",data[2])
        simSetGraphUserData(graphHandle,"error",data[3])
        simSetGraphUserData(graphHandle,"extra",data[4])
    else
        simSetGraphUserData(graphHandle,"command",0)
        simSetGraphUserData(graphHandle,"actual",0)
        simSetGraphUserData(graphHandle,"error",0)
        simSetGraphUserData(graphHandle,"extra",0)
    end
end

function feedback_floor (gap_depth, gap_width, gap_distance)
    if gap_distance == 0 and gap_width == 0 and gap_depth == 0 then
        sim.setObjectPosition(floor_before_gap,-1,  {-0.075, 4.7, -0.18002})
        sim.setObjectPosition(floor_gap,-1,         {-0.075, -0.8, -0.18})
        sim.setObjectPosition(floor_after_gap,-1,   {-0.075, -1.8, -0.18002})
    else
        sim.setObjectPosition(floor_before_gap,-1,  {-0.075, 4.7-gap_distance, -0.18})
        sim.setObjectPosition(floor_gap,-1,         {-0.075, -0.8-gap_distance, -0.18-gap_depth})
        sim.setObjectPosition(floor_after_gap,-1,   {-0.075, -1.8-gap_width-gap_distance, -0.18})
    end

end

-- ROLL --
function feedback_obstacle_roll (obs_distance)
    if obs_distance == 0 then
        --sim.setObjectPosition(obstacle, -1,  {1.4901161193848e-08, -0.5955057144165, -0.03919742628932})
        --print(sim.getObjectPosition(obstacle,-1))
    else
        sim.setObjectPosition(tilt_plate,-1,  {1.4901161193848e-08, -0.5955057144165, -0.03919742628932-obs_distance})
    end

end

-- TILT --
function feedback_obstacle_tilt (obs_distance)
    if obs_distance == 0 then
        sim.setObjectPosition(tilt_plate, -1,  {-0.33800002932549, -0.56700003147125, -0.026})
        --print(sim.getObjectPosition(obstacle,-1))
    else
        sim.setObjectPosition(tilt_plate, -1,  {-0.33800002932549, -0.56700003147125, -0.026-obs_distance})
    end

end

-- OBSTACLE --
function feedback_obstacle_obs (obs_distance)
    if obs_distance == 0 then
        --sim.setObjectPosition(obstacle, -1,  {0.25700005888939, -0.31699994206429, -0.024999992921948})
        print(sim.getObjectPosition(front_plate,-1))
    else
        --sim.setObjectPosition(front_plate,-1,  {0.032, -0.792, -0.055-obs_distance}) -- for stair
        --sim.setObjectPosition(front_plate,-1,  {0.032, -0.792, -0.04}) -- for stair
        sim.setObjectPosition(front_plate,-1,  {0.25700005888939, -0.31699994206429, -0.0275+obs_distance})
    end
end

-- DIRECTION --
function feedback_obstacle_dir (obs_distance)
    if obs_distance == 0 then
        --sim.setObjectPosition(obstacle, -1,  {-1.4970, -0.31699994206429, -1.4560})
        --print(sim.getObjectPosition(obstacle,-1))
    else
        --sim.setObjectPosition(ball,-1,  {0.0-1-obs_distance, -2.25, -0.008}) -- 45 Degree = 1.4970 in x
        sim.setObjectPosition(ball,-1,  {-1.4970, -1, -0.008}) -- 45 Degree = 1.4970 in x
        --sim.setObjectPosition(ball,-1,  {-0.75, -1, -0.008}) -- 45 Degree = 1.4970 in x

    end
end

--[[
Initialization: Called once at the start of a simulation
--]]
if (sim_call_type==sim.childscriptcall_initialization) then

    simulationID=sim.getIntegerSignal("simulationID")
    behaviour=sim.getStringSignal("behaviour")

    if behaviour == "multiple_primitive" then
      behaviour = "multiple"
      primitive = true
    else
      primitive = false
    end

    jointLimit      = true
    jointPos_noise  = false
    com_noise       = false
    mass_noise      = false
    length_noise    = false

    if(behaviour == "obstacle" or behaviour == "multiple" or behaviour == "tilt") then
        com_noise = false
    end

    maxNxtBall      = 17
    terminate       = 0
    old_pan         = 0
    old_theta       = 0
    nxtBall         = 0
    stepCounter     = 0
    mean_vel        = 0
    meanCollision   = 0
    mean_jtor       = 0
    mean_jvel       = 0
    mean_jpower     = 0
    mean_pan        = 0
    mean_tilt       = 0
    mean_roll       = 0
    mean_height     = 0
    mean_slip       = 0
    mean_bbox_x     = 0
    mean_bbox_y     = 0
    mean_bbox_z     = 0
    max_bbox_x      = 0
    max_bbox_y      = 0
    max_bbox_z      = 0
    offset_pan      = 0
    update_count    = 0
    bodyfloor_collisions = 0
    leg_collisions = 0
    position_offset = -10000
    collisionLast1 = false
    collisionLast2 = false
    collisionLast3 = false
    collisionLast4 = false
    collisionLastBF = false
    boolswitch     = true
    ballMoved      = false
    height_arr = {}
    tilt_arr = {}
    oriX_arr = {}
    oriY_arr = {}
    orientation_arr = {}
    circlebreak = false;
    testParameters = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    largest_dist = 0
    collisions_max = 0
    walking_direction = 0
    startingPosition = {}
    behavior_signal = {0, 0, 0, 0, 0}

    gap_control = false
    if gap_control == true then
        floor_before_gap    = sim.getObjectHandle("floor")
        floor_gap           = sim.getObjectHandle("floor_gap")
        floor_after_gap     = sim.getObjectHandle("floor_after")

        gap_depth = math.random(40, 100) * 0.001
        print("Gap size: ".. gap_depth)
        gap_width = math.random(5, 20) * 0.01
        print("Gap width: ".. gap_width)
        gap_distance = math.random(-15, 15) * 0.01
        print("Gap dist.: ".. gap_distance)

        feedback_floor(gap_depth,gap_width,gap_distance)
        --feedback_floor(0,0,0)
        feedback_floor(0.05,0.1,0.03)
    end

    -- TILT
    if(behaviour == "tilt") then
        tilt_plate = sim.getObjectHandle("tilt_plate")
--         obs_distance = (math.random() + math.random(0, 1)) * 0.01
--         print("Obstacle dist.: ".. obs_distance)
--         feedback_obstacle_tilt(obs_distance)
    end

    -- ROLL
    if(behaviour == "roll") then
        tilt_plate = sim.getObjectHandle("tilt_plate")
        obs_distance = (math.random() + math.random(0, 2)) * 0.01
        print("Obstacle dist.: ".. obs_distance)
        feedback_obstacle_roll(obs_distance)
    end

    -- Direction
    if(behaviour == "direction") then
        ball = sim.getObjectHandle("ball")
        --obs_distance = (math.random() + math.random(0, 1.15)-0.5)
        --print("Obstacle dist.: ".. obs_distance)
        --feedback_obstacle_dir(obs_distance)
    end

    if(behaviour == "multiple") then
        ball = sim.getObjectHandle("ball")
    end


    -- Create all handles
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)

    TC_motor0=sim.getObjectHandle("TC0")    -- Handle of the TC motor
    TC_motor1=sim.getObjectHandle("TC1")    -- Handle of the TC motor
    TC_motor2=sim.getObjectHandle("TC2")    -- Handle of the TC motor
    TC_motor3=sim.getObjectHandle("TC3")    -- Handle of the TC motor
    TC_motor4=sim.getObjectHandle("TC4")    -- Handle of the TC motor
    TC_motor5=sim.getObjectHandle("TC5")    -- Handle of the TC motor

    CF_motor0=sim.getObjectHandle("CF0")    -- Handle of the CF motor
    CF_motor1=sim.getObjectHandle("CF1")    -- Handle of the CF motor
    CF_motor2=sim.getObjectHandle("CF2")    -- Handle of the CF motor
    CF_motor3=sim.getObjectHandle("CF3")    -- Handle of the CF motor
    CF_motor4=sim.getObjectHandle("CF4")    -- Handle of the CF motor
    CF_motor5=sim.getObjectHandle("CF5")    -- Handle of the CF motor

    FT_motor0=sim.getObjectHandle("FT0")    -- Handle of the FT motor
    FT_motor1=sim.getObjectHandle("FT1")    -- Handle of the FT motor
    FT_motor2=sim.getObjectHandle("FT2")    -- Handle of the FT motor
    FT_motor3=sim.getObjectHandle("FT3")    -- Handle of the FT motor
    FT_motor4=sim.getObjectHandle("FT4")    -- Handle of the FT motor
    FT_motor5=sim.getObjectHandle("FT5")    -- Handle of the FT motor

    FS0=sim.getObjectHandle("3D_force0")    -- Handle of the FT motor
    FS1=sim.getObjectHandle("3D_force1")    -- Handle of the FT motor
    FS2=sim.getObjectHandle("3D_force2")    -- Handle of the FT motor
    FS3=sim.getObjectHandle("3D_force3")    -- Handle of the FT motor
    FS4=sim.getObjectHandle("3D_force4")    -- Handle of the FT motor
    FS5=sim.getObjectHandle("3D_force5")    -- Handle of the FT motor

    tipHandles = { sim.getObjectHandle("tip_dyn0"),
                   sim.getObjectHandle("tip_dyn1"),
                   sim.getObjectHandle("tip_dyn2"),
                   sim.getObjectHandle("tip_dyn3"),
                   sim.getObjectHandle("tip_dyn4"),
                   sim.getObjectHandle("tip_dyn5")}

    legLengthHandles = { sim.getObjectHandle("leg_length_joint0"),
                   sim.getObjectHandle("leg_length_joint1"),
                   sim.getObjectHandle("leg_length_joint2"),
                   sim.getObjectHandle("leg_length_joint3"),
                   sim.getObjectHandle("leg_length_joint4"),
                   sim.getObjectHandle("leg_length_joint5")}

    IMU=sim.getObjectHandle("Imu")
    IMU_tilt_rel=sim.getObjectHandle("tilt_relative")

    if behaviour == "flip" then
        robot_rel=sim.getObjectHandle("robot_relative")
    else
        robot_rel=sim.getObjectHandle("Graph") -- Graph is used as placeholder (hotfix)
        IMU_tilt_rel=sim.getObjectHandle("tilt_relative")
    end

    distHandle_leg01=sim.getDistanceHandle("leg01")
    distHandle_leg12=sim.getDistanceHandle("leg12")
    distHandle_leg34=sim.getDistanceHandle("leg34")
    distHandle_leg45=sim.getDistanceHandle("leg45")

    distHandle_leg0s=sim.getDistanceHandle("legs0")
    distHandle_leg1s=sim.getDistanceHandle("legs1")
    distHandle_leg2s=sim.getDistanceHandle("legs2")
    distHandle_leg3s=sim.getDistanceHandle("legs3")
    distHandle_leg4s=sim.getDistanceHandle("legs4")
    distHandle_leg5s=sim.getDistanceHandle("legs5")

    --distHandle_BF   =sim.getDistanceHandle("bodyfloor")
    morfHexapod     =sim.getObjectHandle("morfHexapod")
    frontMorf       =sim.getObjectHandle("blinkStick")

    graphHandle     =sim.getObjectHandle("Graph")

    sensor          =sim.getObjectHandle('sensor')

    previousTime=0

    --
    -- Noisy mass
    --
    if mass_noise then
        mass = sim.getShapeMass(morfHexapod)
        sim.setShapeMass(morfHexapod, utils.noise(mass, "gaussian", {0, 0.5}))
    end

    --
    -- Noisy Center of Mass
    --
    if com_noise then
        COM_attach      =sim.getObjectHandle('COM_attach')
        COM_changer     =sim.getObjectHandle('COM_changer')
        COM_position        = sim.getObjectPosition(COM_attach,morfHexapod)
        new_COM_position    = COM_position
        new_COM_position[3] = utils.noise(new_COM_position[3], "gaussian", {0, 0.01})
        new_COM_position[2] = utils.noise(new_COM_position[2], "gaussian", {0, 0.001})
        new_COM_position[1] = utils.noise(new_COM_position[1], "gaussian", {0, 0.001})
        sim.setObjectPosition(COM_attach, morfHexapod, new_COM_position)
    end

    -- Noisy size
    if length_noise then
        variance = 0.0000035
        leg_length_noise =  utils.gaussian(0, variance)

        if(leg_length_noise > 0) then
            leg_length_noise = 0
        end

        print("NOISE IS: " .. leg_length_noise)
        for i = 1, table.getn(legLengthHandles), 1 do
            sim.setJointPosition(legLengthHandles[i], leg_length_noise)
        end
    end

    if(behaviour == "direction" or behaviour == "multiple") then
        walking_direction = utils.calculate_walking_dir(ball, IMU, old_theta)
        old_theta = walking_direction
    end

    -- Check if the required ROS plugin is loaded
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    if (pluginNotFound) then
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        printToConsole('[ERROR] The RosInterface was not found.')
    end

    -- If found then start the subscribers and publishers
    if (not pluginNotFound) then
        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..'morf_sim'..simulationID..'/multi_joint_command','std_msgs/Float32MultiArray','setMotorPositions_cb')
        GraphSub=simROS.subscribe('/'..'morf_sim'..simulationID..'/graph','std_msgs/Float32MultiArray','graph_cb')

        -- Create the publishers
        jointPositionsPub=simROS.advertise('/'..'morf_sim'..simulationID..'/joint_positions','std_msgs/Float32MultiArray')
        jointTorquesPub=simROS.advertise('/'..'morf_sim'..simulationID..'/joint_torques','std_msgs/Float32MultiArray')
        jointVelocitiesPub=simROS.advertise('/'..'morf_sim'..simulationID..'/joint_velocities','std_msgs/Float32MultiArray')
        imuEulerPub=simROS.advertise('/morf_sim'..simulationID..'/euler','geometry_msgs/Vector3')
        testParametersPub=simROS.advertise('/'..'morf_sim'..simulationID..'/testParameters','std_msgs/Float32MultiArray')
    end

    simROS.publish(testParametersPub,{data=testParameters})

    -- WAIT FOR ROS TO START FULLY TO LAUNCH
    printToConsole('[ INFO] Initialized simulation')

    startingPosition = sim.getObjectPosition(IMU,-1)

    -- Set max joint velocitys
    maxSpeed = 1.8;
    sim.setObjectFloatParameter(TC_motor0, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(TC_motor1, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(TC_motor2, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(TC_motor3, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(TC_motor4, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(TC_motor5, sim_jointfloatparam_upper_limit, maxSpeed)

    sim.setObjectFloatParameter(CF_motor0, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(CF_motor1, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(CF_motor2, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(CF_motor3, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(CF_motor4, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(CF_motor5, sim_jointfloatparam_upper_limit, maxSpeed)

    sim.setObjectFloatParameter(FT_motor0, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(FT_motor1, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(FT_motor2, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(FT_motor3, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(FT_motor4, sim_jointfloatparam_upper_limit, maxSpeed)
    sim.setObjectFloatParameter(FT_motor5, sim_jointfloatparam_upper_limit, maxSpeed)
end

--[[
Actuation: This part will be executed in each simulation step
--]]
if (sim_call_type==sim.childscriptcall_actuation) then
    key=0
    message,auxiliaryData=simGetSimulatorMessage()
    while message~=-1 do
        if (message==sim_message_keypress) then
            --print(auxiliaryData[1],auxiliaryData[2],auxiliaryData[3],auxiliaryData[4])
            if (auxiliaryData[1]==string.byte('1')) then
                print("high behavior activated")
                key=1
            elseif (auxiliaryData[1]==string.byte('2')) then
                print("low behavior activated")
                key=2
            elseif (auxiliaryData[1]==string.byte('3')) then
                print("narrow behavior activated")
                key=3
            elseif (auxiliaryData[1]==string.byte('4')) then
                print("pipe behavior activated")
                key=4
            elseif (auxiliaryData[1]==string.byte('5')) then
                print("wall behavior activated")
                key=5
            elseif (auxiliaryData[1]==string.byte('6')) then
                print("walknomed behavior activated")
                key=6
            end
            if(behavior_signal[key]==1) then
                behavior_signal[key]=0
            else
                behavior_signal[key]=1
            end
        end
        message,auxiliaryData=simGetSimulatorMessage()
    end

end

--[[
Sensing: This part will be executed in each simulation step
--]]
if (sim_call_type==sim.childscriptcall_sensing) then

    if (terminate < 0) then
        sim.stopSimulation()
    end
    if(nxtBall == maxNxtBall+1) then
        terminate = -100
    else
        terminate = 100
    end

    robot_orientation = sim.getObjectOrientation(IMU, -1)
    sim.setObjectOrientation(IMU_tilt_rel, -1, {0, 0, robot_orientation[3]})

    not_IMU_robot_orientation = sim.getObjectOrientation(morfHexapod, -1)
    sim.setObjectOrientation(robot_rel, -1, {0, math.abs(not_IMU_robot_orientation[1]), 0})

    if(behaviour == "direction" or behaviour == "multiple") then
        if(behaviour == "multiple") then
            ballTracker()
        end

        if(not ballMoved and behaviour == "direction" and simGetSimulationTime() > 2) then
            obs_distance = (math.random() + math.random(0, 1.15)-0.5)
            print("Obstacle dist.: ".. obs_distance)
            feedback_obstacle_dir(obs_distance)
            ballMoved = true
        end

        walking_direction = utils.calculate_walking_dir(ball, IMU, old_theta)
        old_theta = walking_direction
    end

    -- Publish
    position_array  ={  simGetJointPosition(TC_motor0),simGetJointPosition(CF_motor0),simGetJointPosition(FT_motor0),
                        simGetJointPosition(TC_motor1),simGetJointPosition(CF_motor1),simGetJointPosition(FT_motor1),
                        simGetJointPosition(TC_motor2),simGetJointPosition(CF_motor2),simGetJointPosition(FT_motor2),
                        simGetJointPosition(TC_motor3),simGetJointPosition(CF_motor3),simGetJointPosition(FT_motor3),
                        simGetJointPosition(TC_motor4),simGetJointPosition(CF_motor4),simGetJointPosition(FT_motor4),
                        simGetJointPosition(TC_motor5),simGetJointPosition(CF_motor5),simGetJointPosition(FT_motor5) }

    velocity_array  ={  utils.simGetJointVelocity(TC_motor0),utils.simGetJointVelocity(CF_motor0),utils.simGetJointVelocity(FT_motor0),
                        utils.simGetJointVelocity(TC_motor1),utils.simGetJointVelocity(CF_motor1),utils.simGetJointVelocity(FT_motor1),
                        utils.simGetJointVelocity(TC_motor2),utils.simGetJointVelocity(CF_motor2),utils.simGetJointVelocity(FT_motor2),
                        utils.simGetJointVelocity(TC_motor3),utils.simGetJointVelocity(CF_motor3),utils.simGetJointVelocity(FT_motor3),
                        utils. simGetJointVelocity(TC_motor4),utils.simGetJointVelocity(CF_motor4),utils.simGetJointVelocity(FT_motor4),
                        utils.simGetJointVelocity(TC_motor5),utils.simGetJointVelocity(CF_motor5),utils.simGetJointVelocity(FT_motor5) }

    torque_array    ={  simGetJointForce(TC_motor0),simGetJointForce(CF_motor0),simGetJointForce(FT_motor0),
                        simGetJointForce(TC_motor1),simGetJointForce(CF_motor1),simGetJointForce(FT_motor1),
                        simGetJointForce(TC_motor2),simGetJointForce(CF_motor2),simGetJointForce(FT_motor2),
                        -simGetJointForce(TC_motor3),simGetJointForce(CF_motor3),simGetJointForce(FT_motor3),
                        -simGetJointForce(TC_motor4),simGetJointForce(CF_motor4),simGetJointForce(FT_motor4),
                        -simGetJointForce(TC_motor5),simGetJointForce(CF_motor5),simGetJointForce(FT_motor5) }

    if(behaviour == "obstacle" or behaviour == "multiple") then
        if sim.readProximitySensor(sensor) > 0 then
            torque_array[((0)*3)+1] = 10 -- The two front legs
            torque_array[((1)*3)+1] = 0
            torque_array[((2)*3)+1] = 0
            torque_array[((3)*3)+1] = 10
            torque_array[((4)*3)+1] = 0
            torque_array[((5)*3)+1] = 0
            meanCollision = utils.absmean(meanCollision, 1, update_count)
        else
            torque_array[((0)*3)+1] = 0 -- The two front legs
            torque_array[((1)*3)+1] = 0
            torque_array[((2)*3)+1] = 0
            torque_array[((3)*3)+1] = 0
            torque_array[((4)*3)+1] = 0
            torque_array[((5)*3)+1] = 0
        end
    end

    -- **************** --
    -- Fitness feedback --
    -- **************** --
    linearVelocity, aVelocity=sim.getObjectVelocity(IMU) -- m/s
    objectPosition = sim.getObjectPosition(IMU,-1)
    objectOrientation = sim.getObjectOrientation(IMU,-1)
    objectOrientationForTilt = sim.getObjectOrientation(IMU,IMU_tilt_rel)

    d = objectOrientation[3] - old_pan

    if d > math.pi then
        d=d-2*math.pi
    elseif d < -math.pi then
        d=d+2*math.pi
    end

    headingDirection = old_pan+d
    old_pan = headingDirection

    -- Mean velocity of robot
    mean_vel = utils.absmean(mean_vel, linearVelocity[1], update_count)

    -- Mean power
    mean_jtor   = utils.absmean(mean_jtor, utils.absmean_arr(torque_array), update_count)
    mean_jvel   = utils.absmean(mean_jvel, utils.absmean_arr(velocity_array), update_count)
    mean_jpower = utils.absmean(mean_jpower, mean_jtor * mean_jvel, update_count)

    -- Orientation / Stability
    mean_roll   = utils.absmean(mean_roll, objectOrientation[1], update_count)
    mean_tilt   = utils.absmean(mean_tilt, objectOrientation[2], update_count)
    mean_pan    = utils.absmean(mean_pan, objectOrientation[3]-offset_pan, update_count)

    objectOrientationForRobot = sim.getObjectOrientation(morfHexapod,robot_rel)
--     print("TILT: " .. math.abs(objectOrientationForRobot[3]))

    -- BBox Means
    bboxDim       = getModelBoundingBoxSize(morfHexapod)
    mean_bbox_x   = utils.meanScalar(mean_bbox_x, bboxDim[3], update_count)
    mean_bbox_y   = utils.meanScalar(mean_bbox_y, bboxDim[2], update_count)
    mean_bbox_z   = utils.meanScalar(mean_bbox_z, bboxDim[1], update_count)

    if bboxDim[3] > max_bbox_x then max_bbox_x = bboxDim[3] end
    if bboxDim[2] > max_bbox_y then max_bbox_y = bboxDim[2] end
    if bboxDim[1] > max_bbox_z then max_bbox_z = bboxDim[1] end

    -- Position
    table.insert(height_arr, objectPosition[3])
    table.insert(tilt_arr, objectOrientation[2])

    -- Distance between legs
    max_detect_interleg     = 0.03
    max_detect_intraleg     = 0.005
    max_detect_bodyfloor    = 0.03
    collisionState = {max_detect_interleg,max_detect_interleg,max_detect_interleg,max_detect_interleg,max_detect_intraleg,max_detect_intraleg,max_detect_intraleg,max_detect_intraleg,max_detect_intraleg,max_detect_intraleg,max_detect_bodyfloor}

    -- inter leg.
    result, distance_leg01=sim.handleDistance(distHandle_leg01) -- m
    if distance_leg01 == nil then collisionState[1]=max_detect_interleg else collisionState[1] = distance_leg01 end
    result, distance_leg12=sim.handleDistance(distHandle_leg12) -- m
    if distance_leg12 == nil then collisionState[2]=max_detect_interleg else collisionState[2] = distance_leg12 end
    result, distance_leg34=sim.handleDistance(distHandle_leg34) -- m
    if distance_leg34 == nil then collisionState[3]=max_detect_interleg else collisionState[3] = distance_leg34 end
    result, distance_leg45=sim.handleDistance(distHandle_leg45) -- m
    if distance_leg45 == nil then collisionState[4]=max_detect_interleg else collisionState[4] = distance_leg45 end

    -- intra leg.
    result, distance_leg0s=sim.handleDistance(distHandle_leg0s) -- m
    if distance_leg0s == nil then collisionState[5]=max_detect_intraleg else collisionState[5] = distance_leg0s end
    result, distance_leg1s=sim.handleDistance(distHandle_leg1s) -- m
    if distance_leg1s == nil then collisionState[6]=max_detect_intraleg else collisionState[6] = distance_leg1s end
    result, distance_leg2s=sim.handleDistance(distHandle_leg2s) -- m
    if distance_leg2s == nil then collisionState[7]=max_detect_intraleg else collisionState[7] = distance_leg2s end
    result, distance_leg3s=sim.handleDistance(distHandle_leg3s) -- m
    if distance_leg3s == nil then collisionState[8]=max_detect_intraleg else collisionState[8] = distance_leg3s end
    result, distance_leg4s=sim.handleDistance(distHandle_leg4s) -- m
    if distance_leg4s == nil then collisionState[9]=max_detect_intraleg else collisionState[9] = distance_leg4s end
    result, distance_leg5s=sim.handleDistance(distHandle_leg5s) -- m
    if distance_leg5s == nil then collisionState[10]=max_detect_intraleg else collisionState[10] = distance_leg5s end

    -- body floor.
    result, distance_BF=0--sim.handleDistance(distHandle_BF) -- m
    distance_BF = nil
    if distance_BF == nil then collisionState[11]=max_detect_bodyfloor else collisionState[11] = distance_BF end

    collisionState[1] = 1 - (collisionState[1])   / ( max_detect_interleg ) -- leg01
    collisionState[2] = 1 - (collisionState[2])   / ( max_detect_interleg ) -- leg12
    collisionState[3] = 1 - (collisionState[3])   / ( max_detect_interleg ) -- leg34
    collisionState[4] = 1 - (collisionState[4])   / ( max_detect_interleg ) -- leg45
    collisionState[5] = 1 - (collisionState[5])   / ( max_detect_intraleg ) -- leg0s
    collisionState[6] = 1 - (collisionState[6])   / ( max_detect_intraleg ) -- leg1s
    collisionState[7] = 1 - (collisionState[7])   / ( max_detect_intraleg ) -- leg2s
    collisionState[8] = 1 - (collisionState[8])   / ( max_detect_intraleg ) -- leg3s
    collisionState[9] = 1 - (collisionState[9])   / ( max_detect_intraleg ) -- leg4s
    collisionState[10] = 1 - (collisionState[10]) / ( max_detect_intraleg ) -- leg5s
    collisionState[11] = 1 - (collisionState[11]) / ( max_detect_bodyfloor) -- bodyfloor

    --print(collisionState)
    max_dist = math.max(unpack(collisionState))
    if max_dist > 1 then print("[ ERROR]: Please set max dist correctly") end
    if max_dist > collisions_max then collisions_max = max_dist end

    if(behaviour == "distance") then
        robot_pos    = sim.getObjectPosition(IMU,-1)
        x_diff = startingPosition[1]-robot_pos[1]
        y_diff = startingPosition[2]-robot_pos[2]
        distance = math.sqrt( ((x_diff)*(x_diff)) + ((y_diff)*(y_diff)) )
    else
        positionRobot=sim.getObjectPosition(morfHexapod, -1)
        if(position_offset == -10000) then position_offset = -positionRobot[2] end
        distance = -positionRobot[2] - position_offset -- use negative world y axis
        height_distance = positionRobot[3]
    end


    slip_results = {}
    for i = 1, table.getn(tipHandles), 1 do
        table.insert(slip_results, slip_detector( tipHandles[i], 0.025))
    end

    max_slip = math.max(unpack(slip_results))

    if max_slip ~= -1 then
        mean_slip    = utils.absmean(mean_slip, max_slip, update_count)
    end

    -- Remove transient period
    if simGetSimulationTime() < 1.3 then
        mean_slip=0
        mean_tilt=0
        mean_roll=0
        mean_pan =0
        collisions_max=0
        mean_height=0
        offset_pan = objectOrientation[3]
        height_arr = {0}
        tilt_arr = {0}
    end

    if simGetSimulationTime() > 1 and boolswitch then
        boolswitch = false
        -- Release the robot
         simSetObjectInt32Parameter(morfHexapod, sim_shapeintparam_static, 0)
    end

    testParameters[2]  = mean_slip -- x orientation
    testParameters[3]  = mean_tilt -- x orientation
    testParameters[4]  = mean_roll -- y orientation
    testParameters[5]  = mean_pan -- Heading = z orientation
    testParameters[6]  = collisions_max
    testParameters[7]  = utils.mean(height_arr)
    testParameters[8]  = mean_jpower
    testParameters[9]  = mean_vel
    testParameters[10] = distance
    testParameters[11] = utils.standardDeviation(height_arr)

    -- Read Force sensors
    result, FS0_forceVector, torqueVector=sim.readForceSensor(FS0)
    result, FS1_forceVector, torqueVector=sim.readForceSensor(FS1)
    result, FS2_forceVector, torqueVector=sim.readForceSensor(FS2)
    result, FS3_forceVector, torqueVector=sim.readForceSensor(FS3)
    result, FS4_forceVector, torqueVector=sim.readForceSensor(FS4)
    result, FS5_forceVector, torqueVector=sim.readForceSensor(FS5)
    testParameters[12] = FS0_forceVector[3]
    testParameters[13] = FS1_forceVector[3]
    testParameters[14] = FS2_forceVector[3]
    testParameters[15] = FS3_forceVector[3]
    testParameters[16] = FS4_forceVector[3]
    testParameters[17] = FS5_forceVector[3]
    testParameters[18] = meanCollision
    testParameters[19] = objectOrientation[1] -- ROLL
    testParameters[20] = objectOrientationForTilt[2] -- TILT
    testParameters[21] = headingDirection -- PAN (continous for 2 m_pi)
    testParameters[22] = utils.standardDeviation(tilt_arr)
    testParameters[23] = walking_direction -- Mirrored
    testParameters[24] = mean_bbox_x
    testParameters[25] = mean_bbox_y
    testParameters[26] = mean_bbox_z
    testParameters[27] = math.abs(objectOrientationForRobot[3])
    testParameters[28] = height_distance
    testParameters[29] = sim.getSimulationTime()-- Mirrored
    testParameters[30] = terminate -- Mirrored
    testParameters[31] = max_bbox_x
    testParameters[32] = max_bbox_y
    testParameters[33] = max_bbox_z
    testParameters[34] = behavior_signal[1]
    testParameters[35] = behavior_signal[2]
    testParameters[36] = behavior_signal[3]
    testParameters[37] = behavior_signal[4]
    testParameters[38] = behavior_signal[5]
    testParameters[39] = behavior_signal[6]

    for i=12,17,1 do
        testParameters[i] = math.abs(testParameters[i])
    end

    update_count = update_count + 1

    simROS.publish(jointPositionsPub,{data=position_array})
    simROS.publish(jointVelocitiesPub,{data=velocity_array})
    simROS.publish(jointTorquesPub,{data=torque_array})
    simROS.publish(testParametersPub,{data=testParameters})
end

--[[
Clean up: This part will be executed one time just before a simulation ends
--]]
if (sim_call_type==sim.childscriptcall_cleanup) then
    simROS.publish(jointPositionsPub,{data=position_array})
    simROS.publish(jointVelocitiesPub,{data=velocity_array})
    simROS.publish(jointTorquesPub,{data=torque_array})
    simROS.publish(testParametersPub,{data=testParameters})

    print("+====Objectives====+")
    print("Avg roll:\t"    .. utils.roundToNthDecimal(mean_roll,4))
    print("Avg heading:\t" .. utils.roundToNthDecimal(mean_pan,4))
    print("Avg HeightErr:\t"  .. utils.roundToNthDecimal(math.abs(testParameters[7] - (0.0829*2)) * 15, 4))
    print("Avg power:\t"   .. utils.roundToNthDecimal(mean_jpower,4))
    print("Robot Coll.:\t" .. utils.roundToNthDecimal(collisions_max,5))
    print("Slipping:\t"     .. utils.roundToNthDecimal(mean_slip, 4))
    print("Distance:\t"     .. utils.roundToNthDecimal(distance,4))
    print("Collision:\t"     .. utils.roundToNthDecimal(meanCollision,4))
    print("BBox x:\t"     .. utils.roundToNthDecimal(mean_bbox_x,4))
    print("BBox y:\t"     .. utils.roundToNthDecimal(mean_bbox_y,4))
    print("BBox z:\t"     .. utils.roundToNthDecimal(mean_bbox_z,4))
    print("Avg tilt:\t"    .. utils.roundToNthDecimal(mean_tilt,6))
    print("+================+")

    if(behaviour == "multiple") then
        ball = sim.getObjectHandle("ball")
        result=sim.setShapeColor(ball, nil, 0, {0.95, 0.70, 0.00})
        for i = 0, maxNxtBall do
            ball = sim.getObjectHandle("ball"..i)
            result=sim.setShapeColor(ball, nil, 0, {0.95, 0.70, 0.00})
        end
    end

    -- Reset mass
    if mass_noise then
        sim.setShapeMass(morfHexapod, mass)
    end

    if com_noise then
        sim.setObjectPosition(COM_attach,morfHexapod,COM_position)
    end

    simSetObjectInt32Parameter(morfHexapod, sim_shapeintparam_static, 1)

    -- Terminate remaining local notes
    simROS.shutdownSubscriber(MotorSub)
    simROS.shutdownPublisher(jointTorquesPub)
    simROS.shutdownPublisher(jointVelocitiesPub)
    simROS.shutdownPublisher(jointPositionsPub)
    simROS.shutdownPublisher(testParametersPub)
    simROS.shutdownPublisher(imuEulerPub)

    printToConsole('[ INFO] Lua child script stopped')
end
