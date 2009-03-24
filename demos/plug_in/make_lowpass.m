#! /usr/bin/octave -q

# When alpha = 1, no smoothing occurs

args = argv;
filter_name = args{1};
alpha = str2num(args{2});

a = [1 (1 - alpha)];
b = [alpha 0];

printf('<filter name="%s" type="TransferFunctionFilter">\n', filter_name);
printf('  <params a="%f %f" b="%f %f" />\n', a(1), a(2), b(1), b(2));
printf('</filter>\n');
