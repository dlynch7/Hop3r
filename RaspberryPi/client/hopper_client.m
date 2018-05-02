function hopper_client(port)

%     port='/dev/ttyUSB1';
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end
    fprintf('Opening port  %s....\n',port);
%     mySerial = serial(port, 'BaudRate', 115200, 'FlowControl', 'hardware','Timeout',10); 
    mySerial = serial(port, 'BaudRate', 115200,'Timeout',10); 

    fopen(mySerial);
    nsamples = fscanf(mySerial,'%d');
    
    fprintf('Expecting  %d....\n',nsamples);
    
    fprintf(mySerial,'1');
    
%     fscanf(mySerial,'%d');
    
    Amp = input('\nEnter amplitude ');
    fprintf(mySerial,'%d',Amp);
    fprintf(mySerial,'1');
    fscanf(mySerial,'%d');
    clean = onCleanup(@()fclose(mySerial)); 
    
end
    
%     f = input('\nEnter Frequency ');
% 
%     ts=1/500;
%     T=20;
%     t=0:ts:T;
%     y=Amp*sin(2*pi*f*t);
%     plot(t,y)
%     a=length(y);
%     fprintf('length=  %d....\n',a)
%      
%      for i=1:length(y)-1
%           fprintf('sending  %d....\n',i)   
% %          fprintf(mySerial,'%s\n',x(i));
%      end
%      fprintf('length=  %d....\n',a)
%      
% 
%      exit()
%     
%     
%         
%     
%     
%     fprintf(mySerial,'1\n');
%     data = zeros(nsamples); 
%     
% 
%       for i=1:nsamples
%           data(i) = fscanf(mySerial,'%d'); 
%           fprintf('%d....\n',data(i));   
%       end
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
