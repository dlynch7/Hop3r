function hopper_client_hardcoded(port)

%     port='/dev/ttyUSB1';
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end
    fprintf('Opening port  %s....\n',port);
    mySerial = serial(port, 'BaudRate', 115200,'Timeout',10); 

%     Commenting out for testing without hardware
%     fopen(mySerial);
    nsamples = 1024; % fscanf(mySerial,'%d');
    fprintf('Expecting  %d....\n',nsamples);
%     fprintf(mySerial,'1\n');
%     data = zeros(nsamples); 
        data=zeros(nsamples,3);
    
%     
% Data to be read Boom encoderx3
%                 Motor anglesX3
%                 Joint angleX6
%                 Actual Current
%                 Commanded Current
%                 x,y,angle of foot
%                 commanded Position
%                 Force Sensor reading
%                 IMU Reading
      for i=1:nsamples
          data(i,:) = [i,i+1,i*i];%fscanf(mySerial,'%d'); 
          fprintf('%d %d %d....\n',data(i,1),data(i,2),data(i,3));
%           fprintf('%d %d %d....\n',data[i,0],data[i,1],data[i,2]);  
          pause(0.01);%  for testing without hardware
          
      end
      x=data(:,2);
      y=data(:,3);
      plot(x,y)
      
%     clean = onCleanup(@()fclose(mySerial)); 

    has_quit = false;
    has_quit = true;%For testing without menu
    
    % menu loop
    while ~has_quit
        fprintf('HOPPER INTERFACE\n\n');
        % display the menu options; this list will grow
        fprintf(' a: Home\t b: Calibrate\n c: ChangeGains\t d: Balance\n e: Hop on Point\t f: Kill\n q: Quit\n');
        % read the user's choice
        selection = input('\nENTER COMMAND: ', 's');

        % send the command to the Pi
        fprintf(mySerial,'%c\n',selection);

        % take the appropriate action
        switch selection
            case 'a'
                fprintf('Homing the robot\n')
            case 'b'
                fprintf('Calibrating Sensors\n')
            case 'c'
                fprintf('Changing gains\n')     
                selection = input('\nEnter proportional gain, Kp', 's')
                fprintf(mySerial,'%s\n',selection);
                selection = input('\nEnter Current integral gain, Ki', 's')
                fprintf(mySerial,'%s\n',selection);
                fprintf('The Current gains have been set.')

            otherwise
                fprintf('Invalid Selection %c\n', selection);
        end 
    end

end

