function [str_file] = rotation_interpolation(q,t,initial_key,key,valueKey,str_file)

rot_axon = [num2str(q(1,1)) ' ' num2str(q(2,1)) ' ' num2str(q(3,1))];
%----------------Initial Rotation------------------------
str_file=regexprep(str_file,initial_key,sprintf([rot_axon ' ' num2str(q(4,1))]));
%--------------------------------------------------------

TOTALTIME = t(end);
KeyForAll = t;
NorKeyForAll = KeyForAll/TOTALTIME;

%-------------Interpolate Desired Rotation-------------
str_file=regexprep(str_file,key,sprintf([ num2str(NorKeyForAll(1)) '#NextValue' ])    );
for i=2:length(KeyForAll)
    str_file=regexprep(str_file,'#NextValue',sprintf([', ' num2str(NorKeyForAll(i)) '#NextValue'  ]) );
end
str_file=regexprep(str_file,'#NextValue',' ');

rot_axon = [num2str(q(1,1)) ' ' num2str(q(2,1)) ' ' num2str(q(3,1))];
str_file=regexprep(str_file,valueKey,sprintf([rot_axon ' ' num2str(q(4,1)) '#NextValue' ]));
for i=2:length(KeyForAll)
    rot_axon = [num2str(q(1,i)) ' ' num2str(q(2,i)) ' ' num2str(q(3,i))];
    str_file=regexprep(str_file,'#NextValue',sprintf([',\n' rot_axon ' ' num2str(q(4,i)) '#NextValue' ]) );
end
str_file=regexprep(str_file,'#NextValue',' ');
%--------------------------------------------------------

end