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
    fprintf(mySerial,'1\n');
    data = zeros(nsamples); 
    

      for i=1:nsamples
          data(i) = fscanf(mySerial,'%d'); 
          fprintf('%d....\n',data(i));   
      end
      
    clean = onCleanup(@()fclose(mySerial)); 
    

end
