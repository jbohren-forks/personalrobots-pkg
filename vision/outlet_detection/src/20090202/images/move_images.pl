use Getopt::Std;

getopt("fp");

open(INf, "<".$opt_f) or die "Cannot open file ".$opt_f;

while($line = <INf>)
{
	@line_list = split / /, substr($line, 0, -1);
	$class = $line_list[0];
	$image = $line_list[1];
	print $class, ", ", $image,"\n";

	system("cp ".$opt_p.$image." ".$class."/".$image);	
}
