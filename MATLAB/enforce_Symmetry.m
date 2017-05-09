function [ wp1,wp2,ws1,ws2 ] = enforce_Symmetry(wp1,wp2,ws1,ws2,DesignType)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

if strcmp(DesignType,'Bandpass')
    if ws2 >((wp1*wp2)/ws1)
        ws2=((wp1*wp2)/ws1);
    end
    if ws1 >((wp1*wp2)/ws2)
        ws1=((wp1*wp2)/ws2);
    end
    
end%end if bandpass

if strcmp(DesignType,'Notch')
    if wp1 >((ws1*ws2)/wp2)
        wp1 =((ws1*ws2)/wp2);
    end
    if wp2 >((ws1*ws2)/wp1)
        wp2 =((ws1*ws2)/wp1);
    end
end%end if notch
    
end

