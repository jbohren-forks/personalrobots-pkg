function res = filterserv(req)

res = req._create_response();

method = req.name;

switch method
  case{'butter'} 
    if(length(req.args)==2) %defualt lowpass
      [b,a]=butter(str2num(req.args{1}),str2num(req.args{2}));
    elseif(length(req.args)==3) 
      if(strcmp(req.args{3},'high') || strcmp(req.args{3},'h') || strcmp(req.args{3},'low') || strcmp(req.args{3},'l'))%select type
        [b,a]=butter(str2num(req.args{1}),str2num(req.args{2}), req.args{3});
      else %bandstop
        [b,a]=butter(str2num(req.args{1}),[str2num(req.args{2}), str2num(req.args{3})]);
      endif
    elseif(length(req.args)==4) %bandstop
      [b,a]=butter(str2num(req.args{1}),[str2num(req.args{2}) str2num(req.args{3})], req.args{4});
    else %wrong number of arguments
      res=[];
    end
  case{'cheby1'}
    if(length(req.args)==3) %defualt lowpass
      [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}));
    elseif(length(req.args)==4) 
      if(strcmp(req.args{4},'high') || strcmp(req.args{4},'h') || strcmp(req.args{4},'low') || strcmp(req.args{4},'l'))%select type
        [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}), req.args{4});
      else %bandstop
        [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}), [str2num(req.args{3}), str2num(req.args{4})]);
      endif
    elseif(length(req.args)==5) %bandstop
      [b,a]=cheby1(str2num(req.args{1}),str2num(req.args{2}),[str2num(req.args{3}) str2num(req.args{4})], req.args{5}); 
    else %wrong number of arguments
      res=[];
    end
  case{'cheby2'}
    if(length(req.args)==3) %defualt lowpass
      [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}));
    elseif(length(req.args)==4)
      if(strcmp(req.args{4},'high') || strcmp(req.args{4},'h') || strcmp(req.args{4},'low') || strcmp(req.args{4},'l'))%select type
        [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}), req.args{4});
      else %bandstop
        [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}), [str2num(req.args{3}), str2num(req.args{4})]);
      endif
    elseif(length(req.args)==5) %bandstop
      [b,a]=cheby2(str2num(req.args{1}),str2num(req.args{2}),[str2num(req.args{3}) str2num(req.args{4})], req.args{5}); 
    else %wrong number of arguments
      res=[];
    end
  case{'ellip'}
    if(length(req.args)==4) %defualt lowpass
      [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),str2num(req.args{4}));
    elseif(length(req.args)==5) 
      if(strcmp(req.args{5},'high') || strcmp(req.args{5},'h') || strcmp(req.args{5},'low') || strcmp(req.args{5},'l'))%select type
        [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}),str2num(req.args{4}), req.args{5});
      elseif %bandstop
        [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}), str2num(req.args{3}),[str2num(req.args{4}), str2num(req.args{5})]);
      endif
    elseif(length(req.args)==6) %bandstop
      [b,a]=ellip(str2num(req.args{1}),str2num(req.args{2}),str2num(req.args{3}),[str2num(req.args{4}) str2num(req.args{5})], req.args{6}); 
    else %wrong number of arguments
      res=[];
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
      res=[];
    end  
  otherwise %filter doesn't exist
    res=[];
end

res.a=a;
res.b=b; 



