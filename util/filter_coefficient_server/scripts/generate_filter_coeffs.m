#!/usr/bin/env octave

startup;

__rosoct_unadvertise_service('generate_filter_coeffs');
suc = rosoct_advertise_service('generate_filter_coeffs',@filter_coefficient_server_Filter,@filterserv);

if( ~suc )
    error('failed to advertise service!');
end

while(1)
  __rosoct_worker();
end
