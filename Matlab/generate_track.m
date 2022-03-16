function [track] = generate_track(trackFunc, targetDistance)
    %waypointDistance = [0.9 1.1];
    waypointDistance = [0.5 0.7];
    finished = 0;
    waypoints = 50;
    di = 10;
    changed=1;
    while(changed ==1)
        changed = 0;
        %first calculate the waypoints
        
        pIn = linspace(0,di,waypoints);
%         track = 0.2*[[sin(pIn.*0.01).*pIn+10;pIn];
%         [sin(pIn.*0.01).*pIn-10;pIn];
%         [sin(pIn.*0.01).*pIn;pIn]];
        
    
        trackDistance = 0;
        track=[];
        track = trackFunc(pIn(1));
        for i=2:length(pIn)-1
            track=[track trackFunc(pIn(i))];
            trackDistance=trackDistance+norm([track(5,i-1)-track(5,i) track(6,i-1)-track(6,i)]);
        end
        avgTrackDistance = trackDistance/(length(pIn'));
        if(not(avgTrackDistance >= waypointDistance(1) && avgTrackDistance <= waypointDistance(2)))
            changed=1;
            if(avgTrackDistance >= waypointDistance(1))
                waypoints=waypoints+1;
            else
                waypoints=waypoints-1;
            end
        end
        if(trackDistance<targetDistance*0.95)
            di=di+1;
            changed=1;
        end
        if(trackDistance>targetDistance*1.05)
            di=di-1;
            changed = 1;
        end
        
    end
    %fprintf("Track completed, distance: %d, number of points: %d \r\n", trackDistance, waypoints);

