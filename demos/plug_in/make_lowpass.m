#! /usr/bin/octave -q

# When alpha = 1, no smoothing occurs

args = argv;
filter_name = args{1};
alpha = str2num(args{2});

a = [1 (-(1 - alpha))];
b = [alpha 0];

#printf('<filter name="%s" type="TransferFunctionFilter">\n', filter_name);
#printf('  <params a="%f %f" b="%f %f" />\n', a(1), a(2), b(1), b(2));
#printf('</filter>\n');

printf('<value>');
printf('<array>');
printf('<data>');
printf('<value>');
printf('<struct>');
printf('<member>');
printf('<name>name</name>');
printf('<value>%s</value>', filter_name);
printf('</member>');
printf('<member>');
printf('<name>params</name>');
printf('<value>');
printf('<struct>');
printf('<member>');
printf('<name>a</name>');
printf('<value>');
printf('<array>');
printf('<data>');
printf('<value><double>%f</double></value>', a(1));
printf('<value><double>%f</double></value>', a(2));
printf('</data>');
printf('</array>');
printf('</value>');
printf('</member>');
printf('<member>');
printf('<name>b</name>');
printf('<value>');
printf('<array>');
printf('<data>');
printf('<value><double>%f</double></value>', b(1));
printf('<value><double>%f</double></value>', b(2));
printf('</data>');
printf('</array>');
printf('</value>');
printf('</member>');
printf('</struct>');
printf('</value>');
printf('</member>');
printf('<member>');
printf('<name>type</name>');
printf('<value>MultiChannelTransferFunctionFilterDouble</value>');
printf('</member>');
printf('</struct>');
printf('</value>');
printf('</data>');
printf('</array>');
printf('</value>');
