%% body = openraveros_getbodyinfo(bodyinfo)
%%
%% parses the BodyInfo.msg into a simpler form for octave consumption
function body = openraveros_getbodyinfo(bodyinfo)

body.id = bodyinfo.bodyid;
body.name = bodyinfo.name;
body.type = bodyinfo.type;
body.filename = bodyinfo.filename;

%% extra
body.enabled = bodyinfo.enabled;
body.dof = bodyinfo.dof;
body.T = reshape(cell2mat(bodyinfo.transform.m),[3 4]);

body.jointvalues = cell2mat(bodyinfo.jointvalues);
body.links = zeros(1,length(bodyinfo.links));
for i = 1:length(bodyinfo.links)
    body.links(:,i) = cell2mat(bodyinfo.links{i}.m);
end

body.linknames = bodyinfo.linknames;
body.lowerlimit = cell2mat(bodyinfo.lowerlimit);
body.upperlimit = cell2mat(bodyinfo.upperlimit);
