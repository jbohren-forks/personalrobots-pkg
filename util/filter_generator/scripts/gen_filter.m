#!/usr/bin/env octave

startup;

__rosoct_unadvertise_service('gen_filter');
suc = rosoct_advertise_service('gen_filter',@filter_generator_Filter,@filterserv);

if( ~suc )
    error('failed to advertise service!');
end

while(1)
  __rosoct_worker();
end
