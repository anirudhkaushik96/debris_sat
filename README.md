# debris_sat
IEEE conference paper 


The research paper is based on debris avoidance for satellites in orbit (currently dealing with geostationary satellites).

Problem statement: https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/tree/main/projects/Satellite%20Collision%20Avoidance

Real-time debris tracking is done using: https://www.space-track.org/

Password and username in four.mlx file

My current workflow(optional to work the same way, up to you):

### **Step 1: Set Up Your Environment**
1. **Install Required Toolboxes**
   ```matlab
   >> ver aerospace_toolbox   % Verify Aerospace Toolbox
   >> ver simulink            % Verify Simulink
   >> ver aerospace_blockset  % Verify Aerospace Blockset
   ```
2. **Access Tutorials**
   ```matlab
   >> openExample('aero/ComparisonOfOrbitPropagatorsExample')
   >> openExample('aero/ConstellationModelingWithOrbitPropagatorBlockExample')
   ```

---

### **Step 2: Download Space Debris Data**
1. **Create Space-Track.org Account**  
   Register at [space-track.org](https://www.space-track.org)

2. **Download Debris TLEs**  
   ```matlab
   % Authentication
   credentials = struct('identity','your_email','password','your_password');
   response = webread('https://www.space-track.org/ajaxauth/login', credentials);
   
   % API Query
   tle_url = ['https://www.space-track.org/basicspacedata/query/', ...
              'class/tle_latest/ORDINAL/1/', ...
              'OBJECT_TYPE/DEBRIS/format/tle'];
   options = weboptions('HeaderFields',{'Cookie', ['auth=' response]});
   tle_data = webread(tle_url, options);
   
   % Save Data
   fid = fopen('debris_tles.txt','w');
   fprintf(fid, '%s', tle_data);
   fclose(fid);
   ```

---

### **Step 3: Propagate Debris Orbits (SGP4)**
```matlab
% Create Scenario
startTime = datetime('now');
stopTime = startTime + days(7);
sc = satelliteScenario(startTime, stopTime, 60);

% Add Debris
debris = satellite(sc, 'debris_tles.txt', 'Propagator', 'sgp4');

% Visualize
v = satelliteScenarioViewer(sc);
```

---

### **Step 4: Define Satellite Mission**
1. **Choose Orbit Type**  
   Example: Sun-synchronous orbit for Earth imaging
   ```matlab
   % 700 km altitude SSO
   semiMajorAxis = 7071e3;  % 700 km altitude
   inclination = 98.6;      % SSO inclination
   sat = satellite(sc, semiMajorAxis, 0, inclination, 0, 0, 0, ...
                   'Propagator', 'two-body-keplerian', ...
                   'Name', 'ImagingSat');
   ```

---

### **Step 5: Propagate Combined System**
```matlab
% Propagate all objects
play(sc);

% Extract States
[~, satPos] = states(sat);
[~, debrisPos] = states(debris);
time = sc.TimeHistory;
```

---

### **Step 6: Collision Detection**
```matlab
collisionThreshold = 1000; % 1 km
collisionLog = [];

for i = 1:numel(debris)
    % Calculate relative distance
    relDist = vecnorm(satPos - squeeze(debrisPos(:,:,i)), 2, 2);
    
    % Find collisions
    collisions = find(relDist < collisionThreshold);
    if ~isempty(collisions)
        collisionLog = [collisionLog; 
                        repmat(i, numel(collisions), 1), time(collisions)];
    end
end
```

---

### **Step 7: Collision Avoidance Algorithm**
1. **Maneuver Strategy**  
   Simple radial displacement:
   ```matlab
   function newPos = avoidManeuver(oldPos, deltaV, timeStep)
       % Simple impulsive maneuver
       newPos = oldPos + deltaV * timeStep;
   end
   ```

2. **Automatic Avoidance**  
   ```matlab
   safeDistance = 2000; % 2 km threshold
   for t = 1:numel(time)
       for d = 1:numel(debris)
           currentDist = norm(satPos(t,:) - debrisPos(t,:,d));
           if currentDist < safeDistance
               % Apply maneuver
               satPos(t:end,:) = avoidManeuver(satPos(t:end,:), [0 50 0], 60);
               break;
           end
       end
   end
   ```

---

### **Step 8: Validate Mission Requirements**
1. **Orbit Maintenance Check**
   ```matlab
   finalSMA = norm(satPos(end,:));
   smaError = abs(finalSMA - semiMajorAxis)/semiMajorAxis;
   if smaError > 0.01
       error('Orbit altitude changed by >1%');
   end
   ```

2. **Fuel Usage Analysis**
   ```matlab
   totalDeltaV = sum(vecnorm(diff(satPos), 2, 2));
   fprintf('Total delta-V used: %.2f m/s\n', totalDeltaV);
   ```

---

### **Step 9: Simulink Implementation**
1. **Create Model**  
   ![Collision Avoidance Model](https://mathworks.com/help/examples/aerospace_blks/win64/SatelliteCollisionAvoidanceExample_01.png)

2. **Key Blocks**  
   - Orbit Propagator (SGP4 for debris)
   - Conjunction Analysis
   - Avoidance Maneuver Logic
   - Mission Constraints Check

---

### **Step 10: Test & Validation**
1. **Test Cases**  
   | Scenario | Debris Count | Duration | Expected Result |
   |----------|--------------|----------|------------------|
   | LEO      | 50           | 24 hrs   | 0 collisions     |
   | GEO      | 20           | 1 week   | <3 maneuvers     |

2. **Metrics**  
   ```matlab
   collisionRate = numel(collisionLog)/numel(time);
   maneuverEfficiency = totalDeltaV/numel(unique(collisionLog(:,1)));
   ```

---

### **Final Output Structure**
```matlab
Results = struct(...
    'SimulationDuration', stopTime - startTime,...
    'TotalDebris', numel(debris),...
    'CollisionEvents', collisionLog,...
    'FuelUsed', totalDeltaV,...
    'OrbitStability', smaError);
```

---

### **Key Tips**
1. **Start Simple**  
   Begin with 2-body propagator before adding SGP4 complexity

2. **Batch Processing**  
   Use `parfor` when handling >100 debris objects

3. **Visualization**  
   ```matlab
   % Plot close approach geometry
   closeApproachPlot(satPos, debrisPos(:,:,1), time);
   ```

This approach balances accuracy with computational efficiency, using:
- SGP4 for debris (high precision)
- Two-body for satellite (fast computation)
- Adaptive time steps during close approaches
