// ==========================================
// UI VIEW SWITCHER
// ==========================================
function switchView(targetViewId) {
    // 1. Hide all view sections
    const views = document.querySelectorAll('.view-section');
    views.forEach(view => {
        view.style.display = 'none';
    });

    // 2. Show the requested view
    document.getElementById(targetViewId).style.display = 'block';

    // 3. Update button styling
    const buttons = document.querySelectorAll('.nav-btn');
    buttons.forEach(btn => {
        btn.classList.remove('active');
    });

    // Find the button that was clicked and make it active
    event.currentTarget.classList.add('active');
}

// A. Position Chart (Scatter Plot simulating the Rink)
const ctxPos = document.getElementById('positionChart').getContext('2d');
const positionChart = new Chart(ctxPos, {
    type: 'scatter',
    data: {
        datasets: [{
            label: 'Zamboni Path',
            data: [],
            borderColor: '#3399FF',
            backgroundColor: '#3399FF',
            showLine: false, // Connect the dots!
            borderWidth: 0,
            pointRadius: 1,
            pointHoverRadius: 8
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
            x: { min: -40, max: 40, title: {display: true, text: 'Kaukalon pituus (m)', color: '#aaaaaa'}, 
            grid: { color: 'rgba(255, 255, 255, 0.1)' }, // Faint white gridlines
            ticks: { color: '#888888' }
            },
            y: { min: -20, max: 20, title: {display: true, text: 'Kaukalon leveys (m)', color: '#aaaaaa'}, 
            grid: { color: 'rgba(255, 255, 255, 0.1)' }, 
            ticks: { color: '#888888' }
            }
        },
        plugins: { legend: { display: false } },
        animation: false // Turn off animation for snappy real-time updates
    }
});

// B. Velocity Chart (Line Chart over time)
const ctxVel = document.getElementById('velocityChart').getContext('2d');
const velocityChart = new Chart(ctxVel, {
    type: 'line',
    data: {
        labels: Array(20).fill(''), // 20 blank X-axis labels
        datasets: [{
            label: 'Velocity',
            data: Array(20).fill(0),
            borderColor: '#00E676', /* Neon Green Line */
            backgroundColor: 'rgba(0, 230, 118, 0.1)',
            borderWidth: 3,
            tension: 0.1,
            pointRadius: 0
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
            y: { 
                min: 0, max: 8, 
                title: {display: true, text: 'Nopeus (km/h)', color: '#aaaaaa'},
                grid: { color: 'rgba(255, 255, 255, 0.1)' },
                ticks: { color: '#888888' }
            },
            x: { 
                display: false // Kept hidden as before
            }
        },
        plugins: { legend: { display: false } },
        animation: false
    }
});

// --- 2. MOCK DATA & LOGIC ---
let isRunning = false;
let timeTick = 0;
let waterLevel = 100;
let pathData = [];

// Global variables for Trend Sampling
let latestCmdVel = 0.0;
let latestOdomVel = 0.0;
let mapResolution = 0.1; // Default 5cm
let mapOriginX = 0.0;
let mapOriginY = 0.0;
let mapWidth = 100;
let mapHeight = 100;


// ==========================================
// ROS 2 CONNECTION MANAGER
// ==========================================
const ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', () => {
    console.log('Connected to websocket server.');
    document.getElementById('val-nav').innerText = "Yhdistetty";
    document.getElementById('val-nav').style.color = "#00E676"; // Neon Green
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
    document.getElementById('val-nav').innerText = "Kommunikaatiovirhe";
    document.getElementById('val-nav').style.color = "#ff1744"; // Red
});

ros.on('close', () => {
    console.log('Connection to websocket server closed.');
    document.getElementById('val-nav').innerText = "Ei yhteyttä";
    document.getElementById('val-nav').style.color = "#888";
});

// ==========================================
// CONTROL MODES & TELEOP
// ==========================================
let currentControlMode = 'MANUAL'; // Default for safety
let currentOperationState = 'IDLE';

// The Teleop Publisher
const cmdVelPub = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});

function publishTeleop(linear, angular) {
    if (currentControlMode === 'AUTO') {
        console.warn("Teleop blocked: System is in AUTO mode.");
        return;
    }
    
    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular }
    });
    cmdVelPub.publish(twist);
}

function setControlMode(mode) {
    if (mode === 'MANUAL' && currentOperationState !== 'IDLE' && currentOperationState !== 'HALTED') {
        console.warn("Manual mode locked: Active sequence in progress!");
        return; 
    }


    currentControlMode = mode;
    
    const btnAuto = document.getElementById('btn-mode-auto');
    const btnManual = document.getElementById('btn-mode-manual');
    const btnStart = document.getElementById('btn-start');
    const teleopBtns = document.querySelectorAll('.teleop-btn');

    if (mode === 'AUTO') {
        // Highlight Auto, unhighlight Manual
        btnAuto.classList.add('active-mode');
        btnManual.classList.remove('active-mode');
        
        // Enable Start Sequence button
        btnStart.disabled = false;
        btnStart.style.backgroundColor = '#00E676';
        btnStart.style.color = '#111';

        // Disable all Teleop buttons
        teleopBtns.forEach(btn => btn.disabled = true);
        
    } else if (mode === 'MANUAL') {
        // Highlight Manual, unhighlight Auto
        btnManual.classList.add('active-mode');
        btnAuto.classList.remove('active-mode');
        
        // Disable Start Sequence button
        btnStart.disabled = true;
        btnStart.style.backgroundColor = ''; // Reset to CSS default

        // Enable all Teleop buttons
        teleopBtns.forEach(btn => btn.disabled = false);
    }
}

// ------------------------------------------
// ROS 2 Services (Start sequence)
// ------------------------------------------
const startService = new ROSLIB.Service({
    ros: ros,
    name: '/start_sequence',
    serviceType: 'std_srvs/srv/Trigger' 
});

const stopService = new ROSLIB.Service({
    ros: ros,
    name: '/stop_sequence',
    serviceType: 'std_srvs/srv/Trigger'
});

const resetService = new ROSLIB.Service({
    ros: ros,
    name: '/reset_zamboni',
    serviceType: 'std_srvs/srv/Trigger',
});

function callOperationService(command) {
    const request = new ROSLIB.ServiceRequest({}); 

    if (command === 'START') {
        // Hard-block in JavaScript just in case the HTML disabled attribute fails
        if (currentControlMode === 'MANUAL') {
            alert("Aloitus estetty: Järjestelmä on manuaalitilassa.");
            return;
        }

        startService.callService(request, (result) => {
            console.log("Start Response:", result);
            if (result.success) {
                document.getElementById('val-op').style.color = "#00E676";
                alert("Jäänajo aloitettu") 
            } else {
                alert("START FAILED: " + result.message);
            }
        });
    }
    else if (command === 'STOP') {
        // STOP is universally allowed regardless of mode
        stopService.callService(request, (result) => {
            console.log("Stop Response:", result);
            if (result.success) {
                document.getElementById('val-op').style.color = "#ff1744"; 
                alert("Hätäseis");
            }
        });
        
        // As a safety measure, force vehicle to 0 velocity immediately
        // if (currentControlMode === 'MANUAL') {
        //     publishTeleop(0.0, 0.0);
        // }

    }
    else if (command === 'RESET') {
        resetService.callService(request, (result) => {
            console.log("Reset Response:", result);
            if (result.success) {
                alert("Järjestelmä nollattu.");
            } else {
                alert("Nollaus estetty " + result.message);
            }
        });
    }
}

// Initialize the UI safely to Manual on boot
window.onload = () => {
    setControlMode('MANUAL');
};

const stateListener = new ROSLIB.Topic({
    ros : ros,
    name : '/mission_state',
    messageType : 'std_msgs/String'
});

const stateDict = {
    "IDLE": "Valmiudessa",
    "PHASE_1A": "1A: Ajetaan jäälle",
    "PHASE_1B": "1B: Jäänajon valmistelu",
    "PHASE_2": "2: Jäänajo",
    "PHASE_3A": "3A: Ajetaan pois jäältä",
    "PHASE_3B": "3B: Ajetaan lumentyhjäyspaikalle",
    "PHASE_3C": "3C: Peruutetaan pois lumentyhjäyspaikalta",
    "PHASE_3D": "3D: Ajetaan takaisin talliin",
    "HALTED": "Hätäseis"
};

stateListener.subscribe((message) => {
    let operationState = message.data;
    currentOperationState = operationState;
    // Look up the phase in the dictionary. 
    // The " || 'Virhe' " part is a fallback: if the state isn't found, it defaults to 'Virhe'.
    let operaatio = stateDict[operationState] || "Virhe";
    document.getElementById('val-op').innerText = operaatio;

    const btnReset = document.getElementById('btn-reset');
    const btnManual = document.getElementById('btn-mode-manual');
    const btnStart = document.getElementById('btn-start');

    // state based safety checks
    if (btnReset) {
        if (operationState === 'HALTED') {
            btnReset.disabled = false;
            btnReset.style.backgroundColor = '#ffeb3b';
            btnReset.style.color = '#111';
        } else {
            // Lock the button and strip the inline colors (CSS will take over and gray it out)
            btnReset.disabled = true;
            btnReset.style.backgroundColor = '';
        }
    }
    if (btnManual) {
        if (operationState === 'IDLE' || operationState === 'HALTED') {
            // Unlock manual mode
            btnManual.disabled = false; 
            btnManual.title = ""; // Clear tooltip
        } else {
            // Sequence is running (Phases 1-3). Lock manual mode!
            btnManual.disabled = true;
            btnManual.title = "Pysäytä jäänajo vaihtaaksesi manuaalitilaan"; // Helpful tooltip
        }
    }
    if (btnStart) {
        if (operationState === 'IDLE') {
            btnStart.disabled = false;
        } else {
            btnStart.disabled = true;
        }
    }
});

const velocityListener = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
});

velocityListener.subscribe((message) => {
    // 1. Update text UI
    let currentVel = message.linear.x;
    document.getElementById('val-lin-vel').innerText = currentVel.toFixed(2) + " m/s";
    latestCmdVel = 3.6 * currentVel;
});

const odomListener = new ROSLIB.Topic({
    ros : ros,
    name : '/odometry/filtered',
    messageType : 'nav_msgs/Odometry'
});

odomListener.subscribe((message) => {
    // 1. Raw ROS Coordinates
    let rawX = message.pose.pose.position.x;
    let rawY = message.pose.pose.position.y;

    // 2. Update the Text UI
    let realVel = message.twist.twist.linear.x * 3.6;
    document.getElementById('val-fb-vel').innerText = realVel.toFixed(2) + " km/h";  

    // ==========================================
    // 3. MAIN PAGE: SCATTER PLOT
    // ==========================================
    let plotX = rawX - 34.5;
    let plotY = rawY - 10.25;

    let posData = positionChart.data.datasets[0].data;
    posData.push({x: plotX, y: plotY});
    if (posData.length > 10000) {
        posData.shift(); 
    }
    positionChart.update(); 

    // ==========================================
    // 4. KAAVIOT PAGE: LIVE ZAMBONI ICON
    // ==========================================
    const robotCanvas = document.getElementById('robotCanvas');
    
    if (robotCanvas) {
        const ctxRobot = robotCanvas.getContext('2d');

        // Convert ROS Meters to Canvas Pixels
        let pixelX = (plotX - mapOriginX) / mapResolution;
        
        // Flip the Y-axis!
        let pixelY = mapHeight - ((plotY - mapOriginY) / mapResolution);

        // Clear the previous frame
        ctxRobot.clearRect(0, 0, mapWidth, mapHeight);

        // Extract Yaw (rotation)
        let q = message.pose.pose.orientation;
        let yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        // Draw the Red Triangle Icon
        ctxRobot.save();
        ctxRobot.translate(pixelX, pixelY);
        ctxRobot.rotate(-yaw); 

        ctxRobot.fillStyle = '#ff1744'; 
        ctxRobot.beginPath();
        ctxRobot.moveTo(8, 0);     
        ctxRobot.lineTo(-6, 5);    
        ctxRobot.lineTo(-6, -5);   
        ctxRobot.fill();
        ctxRobot.restore();
    }
    
    latestOdomVel = realVel;
});
// ==========================================
// Occupancy Grid
// ==========================================
const coverageMapCanvas = document.getElementById('coverageMapCanvas');
const ctxCoverage = coverageMapCanvas.getContext('2d');

const gridListener = new ROSLIB.Topic({
    ros : ros,
    name : '/ice_coverage_map',
    messageType : 'nav_msgs/OccupancyGrid'
});

gridListener.subscribe((message) => {
    const width = message.info.width;
    const height = message.info.height;
    const data = message.data; // 1D array of int8 values

    mapResolution = message.info.resolution;
    mapOriginX = message.info.origin.position.x;
    mapOriginY = message.info.origin.position.y;
    mapWidth = width;
    mapHeight = height;

    // 1. Adjust internal canvas resolution to match the ROS grid exactly
    if (coverageMapCanvas.width !== width || coverageMapCanvas.height !== height) {
        coverageMapCanvas.width = width;
        coverageMapCanvas.height = height;

        const robotCanvas = document.getElementById('robotCanvas');
        robotCanvas.width = width;
        robotCanvas.height = height;
    
    }

    // 2. Create an ImageData object to manipulate pixels directly
    const imgData = ctxCoverage.createImageData(width, height);
    const pixelData = imgData.data; // This is a 1D array of RGBA values (4 bytes per pixel)

    // 3. Loop through the ROS Occupancy Grid data
    for (let i = 0; i < data.length; i++) {
        const gridValue = data[i];

        // Determine X, Y coordinates in the ROS grid
        const x = i % width;
        const y = Math.floor(i / width);
        
        // ROS grids have origin at bottom-left, Canvas is top-left.
        // We MUST flip the Y axis so the map draws right-side up!
        const flippedY = (height - 1) - y;
        
        // Calculate the starting index for this pixel in the RGBA array
        const pixelIndex = (flippedY * width + x) * 4;

        // 4. Color Assignment
        if (gridValue === -1) {
            // Unknown / Outside map (Dark Grey to match the panel background)
            pixelData[pixelIndex] = 28;      // R
            pixelData[pixelIndex + 1] = 28;  // G
            pixelData[pixelIndex + 2] = 30;  // B
            pixelData[pixelIndex + 3] = 255; // Alpha (Opacity)
            
        } else if (gridValue === 0) {
            // Free space / Untreated Ice (Light icy white)
            pixelData[pixelIndex] = 230;    
            pixelData[pixelIndex + 1] = 235;
            pixelData[pixelIndex + 2] = 240;
            pixelData[pixelIndex + 3] = 255;
            
        } else {
            // Covered/Occupied (alkuAI Blue - #a1b9ce)
            pixelData[pixelIndex] = 161;     
            pixelData[pixelIndex + 1] = 185;
            pixelData[pixelIndex + 2] = 206;
            pixelData[pixelIndex + 3] = 255;
        }
    }

    // 5. Draw the generated image array directly to the canvas
    ctxCoverage.putImageData(imgData, 0, 0);
});


// Conditioner state
const conditionerListener = new ROSLIB.Topic({
    ros : ros,
    name: '/conditioner_controller/controller_state',
    messageType : 'control_msgs/JointTrajectoryControllerState'
});

conditionerListener.subscribe((message) => {
    let condPos = message.feedback.positions[0];

    // state logic
    let stateText = "Asentovirhe"
    if (condPos > 0.1) {
        stateText = "Alhaalla"
    } else {
        stateText = "Ylhäällä"
    }
    document.getElementById('val-cond').innerText = stateText;
});

// Water valve state
const valveListener = new ROSLIB.Topic({
    ros : ros,
    name: '/water_valve_controller/commands',
    messageType: 'std_msgs/Float64MultiArray'
});

valveListener.subscribe((message) => {
    let valvePos = message.data[0];

    document.getElementById('val-valve').innerText = (valvePos * 100).toFixed(0) + "%";  
});

// Auger state
const augerListener = new ROSLIB.Topic({
    ros : ros,
    name: '/auger_velocity_controller/commands',
    messageType: 'std_msgs/Float64MultiArray'
});

augerListener.subscribe((message) => {
    let augerSpeed = message.data[0];

    document.getElementById('val-auger').innerText = augerSpeed.toFixed(0) + " rad/s";  
});

// Water Tank Level
const tankListener = new ROSLIB.Topic({
    ros : ros,
    name: '/water_tank_level',
    messageType: 'std_msgs/Float64'
});

tankListener.subscribe((message) => {
    // Formatting to 0 decimals so it reads smoothly like "98%"
    document.getElementById('val-water').innerText = message.data.toFixed(0) + " %";  
});

// Water Tank Level
const fuelListener = new ROSLIB.Topic({
    ros : ros,
    name: '/fuel_tank_level',
    messageType: 'std_msgs/Float64'
});

fuelListener.subscribe((message) => {
    // Formatting to 0 decimals so it reads smoothly like "98%"
    document.getElementById('val-fuel').innerText = message.data.toFixed(0) + " %";  
});

const coverageListener = new ROSLIB.Topic({
    ros: ros,
    name: '/ice_coverage_percent',
    messageType: 'std_msgs/Float32'
});

coverageListener.subscribe((message) => {
    // message.data is a float like 45.1234
    document.getElementById('val-coverage').innerText = message.data.toFixed(1) + " %";
});

 // ==========================================
// DIAGNOSTICS & ALARMS LISTENER
// ==========================================
const diagnosticListener = new ROSLIB.Topic({
    ros : ros,
    name : '/diagnostics',
    messageType : 'diagnostic_msgs/DiagnosticArray' // Standard ROS diagnostic type
});

diagnosticListener.subscribe((message) => {
    // Loop through every status reported in the array
    message.status.forEach((status) => {
        
        // Diagnostic Levels: 0=OK, 1=WARN, 2=ERROR, 3=STALE
        // We only care if something is wrong (Level > 0)
        if (status.level > 0) {
            
            // if (status.message.includes("High execution jitter") || 
            //     status.message.includes("No events recorded")) {
            //     return; // Skips the rest of the loop for this specific message
            // }
            // Anti-Spam Filter: Diagnostics publish constantly. 
            // We only want to push to the UI if it's a NEW alarm, 
            // or we will crash the browser with thousands of list items.
            if (alarmQueue.length === 0 || alarmQueue[0].desc !== status.message) {
                
                // Call your existing function!
                // status.message = The error text (e.g., "Goal Rejected")
                // status.name = The node reporting it (e.g., "bt_navigator")
                triggerAlarm(status.message, status.name);
            }
        }
    });
});

// ==========================================
// SYSTEM LOGS LISTENER (/rosout)
// ==========================================
const rosoutListener = new ROSLIB.Topic({
    ros : ros,
    name : '/rosout',
    messageType : 'rcl_interfaces/Log' 
});

const systemList = document.getElementById('system-list');

rosoutListener.subscribe((message) => {
    // message.level dictates severity: 
    // 10 = DEBUG, 20 = INFO, 30 = WARN, 40 = ERROR, 50 = FATAL
    
    // 1. Filter out annoying DEBUG spam from background nodes
    if (message.level < 20) return; 

    // 2. Determine Color and Label
    let colorClass = 'log-info';
    let levelText = 'INFO';

    if (message.level === 30) {
        colorClass = 'log-warn';
        levelText = 'WARN';
    } else if (message.level >= 40) {
        colorClass = 'log-error';
        levelText = 'ERROR';
    }

    const allowedNodes = ['Zamboni_AI', 'conditioner_manager'];

    if (!allowedNodes.includes(message.name)) {
        return;
    }

    // 3. Create a clean timestamp
    const now = new Date();
    const timeStr = now.toLocaleTimeString('en-GB', { hour12: false });

    // 4. Create the new HTML list item
    const logItem = document.createElement('li');
    logItem.className = colorClass;
    
    // Format: [14:05:22] [conditioner_manager] [INFO]: Conditioner engaged...
    logItem.innerHTML = `
        <span style="color: #555;">${timeStr}</span> 
        <span style="color: #888;">${message.name}</span> ${message.msg}
    `;

    // 5. Append to the console
    systemList.prepend(logItem);

    // 6. Memory Management: Prevent the browser from crashing by keeping only the last 100 logs
    if (systemList.childNodes.length > 100) {
        systemList.removeChild(systemList.firstChild);
    }

    // 7. Auto-scroll to the top so the newest log is always perfectly visible
    systemList.scrollTop = 0;
});

// ==========================================
// LOOP 2: LOW-FREQUENCY (1Hz / 1000ms)
// For real-time clocks and elapsed timers
// ==========================================
setInterval(() => {
    
    // 1. Update the System Clock (This always runs, even if stopped)
    const now = new Date();
    const dateStr = now.toLocaleDateString('en-GB'); // Format: DD/MM/YYYY
    const timeStr = now.toLocaleTimeString('en-GB', { hour12: false }); // Format: HH:MM:SS
    document.getElementById('system-clock').innerText = `${dateStr} ${timeStr}`;

    // 2. Update the Elapsed Mission Time (Only runs when Started)
    if (isRunning) {
        secondsElapsed++;
        const mins = String(Math.floor(secondsElapsed / 60)).padStart(2, '0');
        const secs = String(secondsElapsed % 60).padStart(2, '0');
        
        // Uncomment this line if you add an element with id="val-time" to your HTML table!
        // document.getElementById('val-time').innerText = `${mins}:${secs}`;
    }

}, 1000);

// Array to hold our active alarms
let alarmQueue = [];

// Function to trigger a new alarm (Your ROS 2 diagnostic callback will eventually call this)
function triggerAlarm(description, source) {
    const timestamp = new Date().toLocaleTimeString('en-US', { hour12: false });
    
    // Create the alarm object
    const newAlarm = { time: timestamp, desc: description, src: source };
    
    // Push to the front of the array
    alarmQueue.unshift(newAlarm);
    
    // Keep the list from getting infinitely long (e.g., max 50 alarms)
    if (alarmQueue.length > 50) alarmQueue.pop();
    
    updateAlarmUI();
}

// Function to visually update the HTML list
function updateAlarmUI() {
    const alarmListHTML = document.getElementById('alarm-list');
    alarmListHTML.innerHTML = ""; // Clear current list
    
    if (alarmQueue.length === 0) {
        alarmListHTML.innerHTML = `<li style="color: #888;">No active alarms</li>`;
        return;
    }

    // Rebuild the list from the array
    alarmQueue.forEach(alarm => {
        alarmListHTML.innerHTML += `
            <li>
                <span style="color: #888; font-size: 0.9em; margin-right: 10px;">[${alarm.time}]</span>
                <span style="flex-grow: 1;">${alarm.desc}</span>
                <span style="font-weight: bold;">${alarm.src}</span>
            </li>
        `;
    });
}

// Function for the operator to clear the board
function acknowledgeAlarms() {
    alarmQueue = [];
    updateAlarmUI();
}