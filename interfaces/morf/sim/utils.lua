local function roundToNthDecimal(num, n)
    local mult = 10^(n or 0)
    return math.floor(num * mult + 0.5) / mult
end

function mean (t)
    local sum = 0
    for k, v in pairs(t) do
        sum = sum + v
    end
    return sum / #t
end

function standardDeviation (t)
    local squares, avg = 0, mean(t)
    for k, v in pairs(t) do
        squares = squares + ((avg - v) ^ 2)
    end
    local variance = squares / #t
    return math.sqrt(variance)
end

function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) * math.cos(2 * math.pi * math.random()) + mean
end

function absmean (prev_avg, x, n)
    return ((prev_avg * n + math.abs(x)) / (n + 1));
end

function meanScalar (prev_avg, x, n)
    return ((prev_avg * n + x) / (n + 1));
end

function absmean_arr (numlist)
    if type(numlist) ~= 'table' then
        print("Error in absmean_arr")
        return numlist
    end
    num = 0
    table.foreach(numlist,function(i,v) num=num+math.abs(v) end)
    return num / #numlist
end

local function noise(array, noise_model, parameters)
    if noise_model == ("gaussian") then
        -- Generate x gaussian samples and add to the array
        if type(array) == "number" then
            array = array + gaussian(parameters[1], parameters[2])
        else
            for i = 1, table.getn(array), 1 do
                array[i] = array[i] + gaussian(parameters[1], parameters[2])
            end
        end
        return array
    elseif noise_model == ("absgaussian") then
        -- Generate x gaussian samples and add to the array
        if type(array) == "number" then
            array = array + math.abs(gaussian(parameters[1], parameters[2]))
        else
            for i = 1, table.getn(array), 1 do
                array[i] = array[i] + math.abs(gaussian(parameters[1], parameters[2]))
            end
        end
        return array
    elseif noise_model == ("uniform") then -- not implemented
        parameters[1] = parameters[1] * 1000 -- to fix float issue
        parameters[2] = parameters[2] * 1000 -- to fix float issue

        if type(array) == "number" then
            array = array + (math.random(parameters[1], parameters[2])*0.001)
        else
            for i = 1, table.getn(array), 1 do
                array[i] = array[i] + (math.random(parameters[1], parameters[2])*0.001)
            end
        end
        return array
    elseif noise_model == ("other") then -- not implemented
        return array
    end
end

function simGetJointVelocity (jointHandle)
    res,velocity=simGetObjectFloatParameter(jointHandle,2012)
    return  velocity
end

function slip_detector( object_handle, vel_threshold )
    index=0
    objectsInContact,contactPt,forceDirectionAndAmplitude=sim.getContactInfo(sim.handle_all,object_handle,index)

    linearVelocity, angularVelocity = sim.getVelocity(object_handle)
    absLinearVelocity = math.sqrt((linearVelocity[1]*linearVelocity[1]) + (linearVelocity[2]*linearVelocity[2]))

    if objectsInContact then
        if absLinearVelocity > vel_threshold then return 1 end
        return 0
    else return 0
    end
end

function jointLimiter(joint_value, joint_min, joint_max)
    joint_min = joint_min * math.pi/180
    joint_max = joint_max * math.pi/180

    if jointLimit == true then
        if joint_value > joint_max then
            --print("joint violation")
            return joint_max
        elseif joint_value < joint_min then
            --print("joint violation")
            return joint_min
        end
    end

    return joint_value
end

function calculate_walking_dir (ball, IMU, old_theta)
    obstacle_pos = sim.getObjectPosition(ball,-1)
    robot_pos    = sim.getObjectPosition(IMU,-1)

    x_diff = obstacle_pos[1]-robot_pos[1]
    y_diff = robot_pos[2]-obstacle_pos[2]

    theta = math.atan2(x_diff, y_diff)

    d = theta - old_theta

    if d > math.pi then
        d=d-2*math.pi
    elseif d < -math.pi then
        d=d+2*math.pi
    end

    return old_theta+d
end

function getObjectBoundingBoxSize(handle)
    local s={}
    local r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_max_x)
    local r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_min_x)
    s[1]=m-n
    local r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_max_y)
    local r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_min_y)
    s[2]=m-n
    local r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_max_z)
    local r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_min_z)
    s[3]=m-n
    return s
end

function getModelBoundingBoxSize(handle)
    local s={}
    local r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_modelbbox_max_x)
    local r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_modelbbox_min_x)
    s[1]=m-n
    local r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_modelbbox_max_y)
    local r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_modelbbox_min_y )
    s[2]=m-n
    local r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_modelbbox_max_z)
    local r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_modelbbox_min_z)
    s[3]=m-n
    return s
end

return { roundToNthDecimal = roundToNthDecimal,
         mean = mean,
         meanScalar = meanScalar,
         standardDeviation = standardDeviation,
         gaussian = gaussian,
         absmean = absmean,
         absmean_arr = absmean_arr,
         noise = noise,
         simGetJointVelocity = simGetJointVelocity,
         slip_detector = slip_detector,
         jointLimiter = jointLimiter,
         calculate_walking_dir = calculate_walking_dir}