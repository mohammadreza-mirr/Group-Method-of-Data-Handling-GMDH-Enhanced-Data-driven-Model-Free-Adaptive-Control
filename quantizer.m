function [quantizedzz] = quantizer(ysend, tetaqantizer, basicZ0, resolution)
for i= 1:resolution
    
    zi(i)= basicZ0 * (tetaqantizer^i);
    
    end
    delta =(1-tetaqantizer)/(1+tetaqantizer);
    quantizedzz=0; 
    for j=1:resolution
     
    if  (ysend  > (1/(1+delta))*zi(j))
            
            quantizedzz = max(zi(j),quantizedzz);
          
    else 
            quantizedzz = 0;
    end
    
    
end
end

