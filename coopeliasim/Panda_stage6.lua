-- Stage 6 scene truth: ten unequal serial bending joints, independent of MATLAB PCC.
-- CoppeliaSim child script attached to Panda in the stage6 scene copy.
local phi,theta=0,0
local joints={}
local disks={}
local disk1,camera,flange,sensor
local lengths={0.004,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005}
local weights={0.04,0.05,0.06,0.07,0.08,0.09,0.10,0.12,0.16,0.23}

local function mul(A,B)
    local C={}
    for i=0,2 do
        for j=0,2 do
            local v=0
            for k=0,2 do v=v+A[4*i+k+1]*B[4*k+j+1] end
            C[4*i+j+1]=v
        end
        local v=A[4*i+4]
        for k=0,2 do v=v+A[4*i+k+1]*B[4*k+4] end
        C[4*i+4]=v
    end
    return C
end

local function rz(a)
    local c,s=math.cos(a),math.sin(a)
    return {c,-s,0,0,s,c,0,0,0,0,1,0}
end
local function ry(a)
    local c,s=math.cos(a),math.sin(a)
    return {c,0,s,0,0,1,0,0,-s,0,c,0}
end
local function tz(d)
    return {1,0,0,0,0,1,0,0,0,0,1,d}
end

local function updateFlexPose()
    local M=sim.getObjectMatrix(disk1,-1)
    local Rp=rz(phi)
    local Rm=rz(-phi)
    for i=1,10 do
        local half=ry(theta*weights[i]/2)
        M=mul(M,mul(mul(mul(mul(Rp,half),Rm),tz(lengths[i])),mul(mul(Rp,half),Rm)))
        if i<=9 then sim.setObjectMatrix(disks[i],-1,M) end
    end
    sim.setObjectMatrix(camera,-1,mul(M,tz(0.008)))
end

function updateState_function(inInts,inFloats,inStrings,inBuffer)
    if #inFloats~=9 then error('Stage 6 expects exactly nine joint targets') end
    for i=1,7 do sim.setJointTargetPosition(joints[i],inFloats[i]) end
    phi=inFloats[8]
    theta=inFloats[9]
    return {},{phi,theta},{},''
end

function getStage6State_function(inInts,inFloats,inStrings,inBuffer)
    return {},{phi,theta},{},''
end

function getStage6Observation_function(inInts,inFloats,inStrings,inBuffer)
    local out={}
    for i=1,7 do out[#out+1]=sim.getJointPosition(joints[i]) end
    out[#out+1]=phi;out[#out+1]=theta
    local p07=sim.getObjectPosition(flange,-1)
    local p08=sim.getObjectPosition(disk1,-1)
    local cam=sim.getObjectMatrix(sensor,-1)
    for i=1,3 do out[#out+1]=p07[i] end
    for i=1,3 do out[#out+1]=p08[i] end
    for i=1,12 do out[#out+1]=cam[i] end
    return {},out,{},''
end

function sysCall_init()
    for i=1,7 do joints[i]=sim.getObjectHandle('Panda_joint'..i) end
    disk1=sim.getObjectHandle('Joint_disk01')
    flange=sim.getObjectHandle('Panda_attachment')
    for i=1,9 do disks[i]=sim.getObjectHandle(string.format('Joint_disk%02d',i+1)) end
    camera=sim.getObjectHandle('Camera')
    sensor=sim.getObjectHandle('blobDetectionCamera_camera')
    updateFlexPose()
end

function sysCall_sensing()
    -- Use the arm pose after the synchronous physics step.
    updateFlexPose()
end
