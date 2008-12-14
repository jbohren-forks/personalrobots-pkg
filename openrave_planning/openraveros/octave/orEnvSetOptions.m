% success = orEnvSetOptions(options)
%
% A string of various environment options. Example usage:
% orEnvSetOptions('publishanytime 1');
%
% Current options:
% - simulation [start/stop] [time_step] - toggles the internal simulation loop, ie all the calls to SimulationStep. If time_step is specified, will set the simulation time step for all objects. Note that this is not tied to real time at all, how fast the simulation goes in reality depends on complexity of the scene and the physics engine being used.
% - physics engine_name - switches the physics engine to another one with id 'engine_name'
% - collision checker_name - switches the collision checker to a new one with id 'checker_name'
% - gravity [x y z] - changes to gravity vector
% - publishanytime [1/0] - switch between publishing the body transformations
%           to the GUI anytime or only between stepsimulation and server  messsages.
%           When publishing anytime, the GUI will reflect the body movements after every
%           move. This is useful when visualizing internal C++ states. When off, the GUI
%           will only reflect the state of robots after all calls to stepsimulation and
%           server send messages have been done. The default is off.
% - debug [debug level] - toggles debugging messages by RAVELOG.
%                         0  - only RAVEPRINT statements show
%                         1+ - RAVELOG statements with various debug levels show
% - quit - closes the openrave instance
function success = orEnvSetOptions(options)

session = openraveros_getglobalsession();
req = openraveros_env_set();

[cmd, rem] = strtok(options, ' ');
cmd = strtrim(cmd);
switch(cmd)
    case 'simulation'
        req.setmask = req.Set_Simulation();
        [cmd,rem] = strtok(cmd);
        cmd = strtrim(cmd);
        
        if( strcmp(cmd,'start') )
            req.sim_action = req.SimAction_Start();
        elseif( strcmp(cmd,'stop') )
            req.sim_action = req.SimAction_Stop();
        elseif( strcmp(cmd,'timestep') )
            req.sim_action = req.SimAction_Timestep();
        end

        n = str2num(rem);
        if( ~isempty(n) )
            req.sim_timestep = n;
        end
        
    case 'physics'
        req.setmask = req.Set_PhysicsEngine();
        req.physicsengine = strtrim(rem);
    case 'collision'
        req.setmask = req.Set_CollisionChecker();
        req.collisionchecker = strtrim(rem);
    case 'viewer'
        req.setmask = req.Set_Viewer();
        req.viewer = strtrim(rem);
    case 'gravity'
        req.setmask = req.Set_Gravity();
        req.gravity(1:3) = mat2cell(str2num(rem),1,[1 1 1]);
    case 'publishanytime'
        req.setmask = req.Set_PublishAnytime();
        req.publishanytime = str2num(rem);
    case 'debug'
        req.setmask = req.Set_DebugLevel();
        req.debuglevel = str2num(rem);
    otherwise
        display('unknown command');
end

res = rosoct_session_call(session.id,'env_set',req);
success = ~isempty(res);
