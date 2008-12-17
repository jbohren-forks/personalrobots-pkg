function [success,resdata] = filterserv(reqdata)

[req,res] = rosoct_Filter();

req = req._deserialize(reqdata);

if(req.name =='butter')
  if(length(req.args)<3)

    [b,a]=butter(str2num(req.args{1}),str2num(req.args{2}));
  else
    [b,a]=butter(str2num(req.args{1}),str2num(req.args{2}), req.args{3});
  end
  res.a=a;
  res.b=b;
else
  res.a=[];
  res.b=[];
end

success = 1;
resdata = res._serialize(res,0);

