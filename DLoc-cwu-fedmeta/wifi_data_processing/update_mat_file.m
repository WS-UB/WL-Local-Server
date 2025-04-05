dataset = "jacobs_Aug16_1";
filename = fullfile("C:\Users\kanis\DLoc_datasets\channels_release", "channels_" + dataset + ".mat");

% Load the data
data = load(filename);

% Modify ap_aoa (replace with your desired values)
data.ap_aoa = [0, pi, pi/2, -pi/2]; 

% Save the updated struct back to the file
save(filename, '-struct', 'data');
