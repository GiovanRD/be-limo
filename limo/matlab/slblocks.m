function blkStruct = slblocks
%SLBLOCKS Show limo_lib in the Simulink Library Browser.

Browser.Library = 'limo_lib';
Browser.Name = 'LIMO (ENSEM)';

blkStruct.Browser = Browser;

end
