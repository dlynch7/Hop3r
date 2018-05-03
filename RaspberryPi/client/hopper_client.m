function hopper_client(port)

%     port='/dev/ttyUSB1';
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end
    
%     clean = onCleanup(@()fclose(port));
    
    fprintf('Opening port  %s....\n',port);
%     mySerial = serial(port, 'BaudRate', 115200, 'FlowControl', 'hardware','Timeout',10); 
    mySerial = serial(port, 'BaudRate', 115200,'Timeout',10); 

    fopen(mySerial);
    fprintf('serial port opened!\n');
    nsamples = fscanf(mySerial,'%d');
    
    fprintf('Expecting %d samples\n',nsamples);
    
    runPermission = 1;
%     fprintf(mySerial,'1');
    fprintf(mySerial,'%d\n',runPermission);
    
%     fscanf(mySerial,'%d');
    
    Amp = input('\nEnter amplitude: ');
%     fprintf(mySerial,'%d\n',Amp);
%     fprintf(mySerial,'1\n');
%     fscanf(mySerial,'%d'); 
    

    
    f = input('\nEnter frequency: ');

    ts = 1/500;
    T = ts*nsamples;
    t = 0:ts:T;
    y_mA = int16(Amp*1000*sin(2*pi*f*t));
    
    figure;
    plot(t,y_mA)
    

     for i=1:length(y_mA)-1
        str = sprintf('%d',y_mA(i));
        fprintf('sending  y(%d) = %d\n',i,y_mA(i))
        fprintf(mySerial,'%s\n',str);
     end
     
     fprintf("done sending\n");
     flushinput(mySerial);
%      fscanf(mySerial); % flush the serial buffer
     
     
%      ser_check = fscanf(mySerial,'%d');
%      fprintf("ser_check = %d\n",ser_check);
%      
%      if (fscanf(mySerial,'%d') ~= nsamples)
%         writePermission = 0;
%         fprintf(mySerial,'%d\n',writePermission);
%         fprintf("Serial comm experienced an error. Program ending.\n"); 
%         return;
%      end
     
     fscanf(mySerial);
     
%      flushoutput(mySerial);
     flushinput(mySerial); % flush the serial buffer
   
%     
%         
    writePermission = 1;
     fprintf(mySerial,'%d\n',writePermission);
%     
%     fprintf(mySerial,'1\n');
%     data = zeros(nsamples); 
%     
% 
      fscanf(mySerial);
        for i=1:nsamples
%           str = fscanf(mySerial,'%s');
%           fprintf('%s\n',str);
        data(i,1:3) = fscanf(mySerial,'%d %d %d');
        fprintf('%d: %d %d %d\n',[i-1,data(i,1:3)]);
      end
      
      fprintf("done reading\n");
      
      figure;
      plot(data(:,1:3));
      title('Received data')
      
end 
%       
%     clean = onCleanup(@()fclose(mySerial)); 
% 
%     has_quit = false;
%     has_quit = True;%For testing without menu
%     
%     % menu loop
%     while ~has_quit
%         fprintf('HOPPER INTERFACE\n\n');
%         % display the menu options; this list will grow
%         fprintf(' a: Home\t b: Calibrate\n c: ChangeGains\t d: Balance\n e: Hop on Point\t f: Kill\n q: Quit\n');
%         % read the user's choice
%         selection = input('\nENTER COMMAND: ', 's');
% 
%         % send the command to the Pi
%         fprintf(mySerial,'%c\n',selection);
% 
%         % take the appropriate action
%         switch selection
%             case 'a'
%                 fprintf('Homing the robot\n')
%             case 'b'
%                 fprintf('Calibrating Sensors\n')
%             case 'c'
%                 fprintf('Changing gains\n')     
%                 selection = input('\nEnter proportional gain, Kp', 's')
%                 fprintf(mySerial,'%s\n',selection);
%                 selection = input('\nEnter Current integral gain, Ki', 's')
%                 fprintf(mySerial,'%s\n',selection);
%                 fprintf('The Current gains have been set.')
% 
%             otherwise
%                 fprintf('Invalid Selection %c\n', selection);
%         end 
%     end
% 
% end
% 
