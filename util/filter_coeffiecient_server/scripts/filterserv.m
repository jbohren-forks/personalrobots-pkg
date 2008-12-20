function res = filterserv(req)

res = req._create_response();

method = req.name;

switch method
  case{'butter'} 
    if(length(req.args)==2) %defualt lowpass
      [b,a]=butter(str2num(req.args{1}),str2num(req.args{2}));
    elseif(length(req.args)==3) %select type
      [b,a]=butter(str2num(req.args{1}),str2num(req.args{2}), req.args{3});
    elseif(length(req.args)==4) %bandstop
      [b,a]=butter(str2num(req.args{1}),[str2num(req.args{2}) str2num(req.args{3})], req.args{4});
    else %wrong number of arguments
      a=[];
      b=[];
    end
  case{'cheby1'}
    if(length(req.args)==3) %defualt lowpass
      [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}));
    elseif(length(req.args)==4) %select type
      [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}), req.args{4});
    elseif(length(req.args)==5) %bandstop
      [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}),[str2num(req.args{3}) str2num(req.args{4})], req.args{5}); 
    else %wrong number of arguments
      a=[];
      b=[];
    end
  case{'cheby2'}
    if(length(req.args)==3) %defualt lowpass
      [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}));
    elseif(length(req.args)==4) %select type
      [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}), req.args{4});
    elseif(length(req.args)==5) %bandstop
      [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}),[str2num(req.args{3}) str2num(req.args{4})], req.args{5}); 
    else %wrong number of arguments
      a=[];
      b=[];
    end
  case{'ellip'}
    if(length(req.args)==4) %defualt lowpass
      [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),str2num(req.args{4}));
    elseif(length(req.args)==5) %select type
      [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}),str2num(req.args{4}), req.args{5});
    elseif(length(req.args)==6) %bandstop
      [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),[str2num(req.args{4}) str2num(req.args{5})], req.args{6}); 
    else %wrong number of arguments
      a=[];
      b=[];
    end
  case{'invfreqz'}
    if(length(req.args)==4) %defualt 
      [b,a]=invfreqz(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),str2num(req.args{4}));
    elseif(length(req.args)==5) %fit-errors vs. frequency
      [b,a]=invfreqz(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}),str2num(req.args{4}), str2num(req.args{5}));
    elseif(length(req.args)==6) %convergence
      [b,a]=invfreqz(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),[str2num(req.args{4}) str2num(req.args{5})], str2num(req.args{6}));
    elseif(length(req.args)==7) %convergence and tolerance
      [b,a]=invfreqz(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),[str2num(req.args{4}) str2num(req.args{5})], str2num(req.args{6}), str2num(req.args{7}));   
    else %wrong number of arguments
      a=[];
      b=[];
    end  
  otherwise %filter doesn't exist
    a=[];
    b=[];
end

res.a=a;
res.b=b; 


