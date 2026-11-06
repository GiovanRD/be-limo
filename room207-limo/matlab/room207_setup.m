function room207_setup()
%ROOM207_SETUP Add the Room 207 and LIMO MATLAB folders to the path.
%
%   room207_setup()
%
%   Run once per MATLAB session before opening the Room 207 models. The
%   models also call it when they load.
%
%   The LIMO library (limo/matlab) is looked for next to this repository,
%   as in the robotic_lab layout:
%
%       robotic_lab/
%       ├── room207-limo/matlab/   (this folder)
%       ├── limo/matlab/           (limo_lib.slx and the +limo package)
%       └── environment/           (QCar track map)
%
%   Set the environment variable LIMO_MATLAB_PATH to use another location.

here = fileparts(mfilename('fullpath'));
addpath(here);

root = fileparts(fileparts(here));

candidates = {
    getenv('LIMO_MATLAB_PATH')
    fullfile(root, 'limo', 'matlab')
    fullfile(root, 'limo_ros2', 'matlab')
};

for i = 1:numel(candidates)
    folder = candidates{i};

    if ~isempty(folder) && isfile(fullfile(folder, 'limo_lib.slx'))
        addpath(folder);
        return
    end
end

error('room207:setup', ...
    ['LIMO library not found. Clone the limo repository next to ' ...
     'room207-limo, or set LIMO_MATLAB_PATH to its matlab folder.']);

end
