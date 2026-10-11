local done=false
function sysCall_nonSimulation()
    if done then return end
    done=true
    local logPath='E:/6thWork/simulation/matlab/Experiment/stage6_results/scene_patch_log.txt'
    local source='E:/6thWork/simulation/coopeliasim/Panda_robot_flexible_endoscope_MDH.ttt'
    local output='E:/6thWork/simulation/coopeliasim/Panda_robot_flexible_endoscope_stage6.ttt'
    local scriptPath='E:/6thWork/simulation/coopeliasim/Panda_stage6.lua'
    local ok,result=pcall(function()
        sim.loadScene(source)
        local panda=sim.getObjectHandle('Panda')
        local child=sim.getScriptAssociatedWithObject(panda)
        assert(child>=0,'Panda child script not found')
        local f=assert(io.open(scriptPath,'rb'))
        local text=f:read('*a')
        f:close()
        sim.setScriptText(child,text)
        sim.saveScene(output)
        local g=assert(io.open(logPath,'w'))
        g:write('saved=',output,'\nchild=',tostring(child),'\nbytes=',tostring(#text),'\n')
        g:close()
    end)
    if not ok then
        local g=assert(io.open(logPath,'w'))
        g:write('error=',tostring(result),'\n')
        g:close()
    end
    sim.quitSimulator()
end
