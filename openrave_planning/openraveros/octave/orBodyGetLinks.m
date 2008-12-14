% values = orBodyGetLinks(bodyid)
%
% Returns the transformations of all the body's links in a 12 x L matrix. Where L
% is the number of links and each column is a 3x4 transformation
% (use T=reshape(., [3 4]) to recover).
% T * [X;1] = Xnew

function values = orBodyGetLinks(bodyid)
session = openraveros_getglobalsession();
req = openraveros_body_getlinks();
req.bodyid = bodyid;
res = rosoct_session_call(session.id,'body_getlinks',req);

if(~isempty(res))
    values = zeros(1,length(res.links));
    for i = 1:length(res.links)
        values(:,i) = cell2mat(res.links{i}.m);
    end
else
    values = [];
end

