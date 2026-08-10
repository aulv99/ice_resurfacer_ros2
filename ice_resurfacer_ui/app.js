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
            x: { min: -30, max: 30, title: {display: true, text: 'Kaukalon pituus (m)', color: '#aaaaaa'}, 
            grid: { color: 'rgba(255, 255, 255, 0.1)' }, // Faint white gridlines
            ticks: { color: '#888888' }
            },
            y: { min: -15, max: 15, title: {display: true, text: 'Kaukalon leveys (m)', color: '#aaaaaa'}, 
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

function callOperationService(command) {
    const request = new ROSLIB.ServiceRequest({}); // Trigger services don't need data sent

    if (command === 'START') {
        startService.callService(request, (result) => {
            console.log("Start Response:", result);
            if (result.success) {
                // Optional: Make the UI visually confirm it started
                document.getElementById('val-op').style.color = "#00E676"; 
            } else {
                // If the robot refused to start, pop up a browser alert!
                alert("START FAILED: " + result.message);
            }
        });
    }
    else if (command === 'STOP') {
        stopService.callService(request, (result) => {
            console.log("Stop Response:", result);
            if (result.success) {
                document.getElementById('val-op').style.color = "#ff1744"; 
                alert("ZAMBONI HALTED!");
            }
        });
    }
}

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
    "PHASE_3D": "3D: Ajetaan takaisin talliin"
};

stateListener.subscribe((message) => {
    let operationState = message.data;
    
    // Look up the phase in the dictionary. 
    // The " || 'Virhe' " part is a fallback: if the state isn't found, it defaults to 'Virhe'.
    let operaatio = stateDict[operationState] || "Virhe";

    document.getElementById('val-op').innerText = operaatio;
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
    let posX = message.pose.pose.position.x;
    let posY = message.pose.pose.position.y;

    let realVel = message.twist.twist.linear.x * 3.6;
    document.getElementById('val-fb-vel').innerText = realVel.toFixed(2) + " km/h";  

    let posData = positionChart.data.datasets[0].data;
    posData.push({x: posX, y: posY});
    if (posData.length > 10000) {
        posData.shift(); 
    }
    positionChart.update();

    let velData = velocityChart.data.datasets[0].data;
    velData.push(realVel);
    velData.shift(); 
    velocityChart.update()
    latestOdomVel = realVel;
});

// ==========================================
// MULTI-TREND VIEWER (Page 3)
// ==========================================
const ctxTrend = document.getElementById('multiTrendChart').getContext('2d');
const multiTrendChart = new Chart(ctxTrend, {
    type: 'line',
    data: {
        labels: Array(100).fill(''), // 100 data points on the X-axis
        datasets: [
            {
                label: 'Nopeusohje (km/h)', // Dataset 0
                data: Array(100).fill(0),
                borderColor: '#3399FF', // alkuAI Blue
                borderWidth: 2,
                tension: 0.2,
                pointRadius: 0
            },
            {
                label: 'Todellinen nopeus (km/h)', // Dataset 1
                data: Array(100).fill(0),
                borderColor: '#00E676', // Neon Green
                borderWidth: 2,
                tension: 0.2,
                pointRadius: 0
            }
        ]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
            y: {  min: -5, max: 10,
                title: {display: true, text: 'Nopeus (km/h)', color: '#aaaaaa'},
                grid: { color: 'rgba(255, 255, 255, 0.1)' },
                ticks: { color: '#888888' }
            },
            x: { display: false }
        },
        plugins: { 
            legend: { 
                display: true, 
                labels: { color: '#e0e0e0' } 
            } 
        },
        animation: false
    }
});

setInterval(() => {
    // 1. Grab the arrays for both lines
    let cmdData = multiTrendChart.data.datasets[0].data;
    let odomData = multiTrendChart.data.datasets[1].data;

    // 2. Push the latest values simultaneously
    cmdData.push(latestCmdVel);
    odomData.push(latestOdomVel);

    // 3. Shift the oldest data off the front to maintain the 100-point window
    cmdData.shift();
    odomData.shift();

    // 4. Update the chart once
    multiTrendChart.update();
}, 200);


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
    systemList.appendChild(logItem);

    // 6. Memory Management: Prevent the browser from crashing by keeping only the last 100 logs
    if (systemList.childNodes.length > 100) {
        systemList.removeChild(systemList.firstChild);
    }

    // 7. Auto-scroll to the bottom so the newest log is always visible
    systemList.scrollTop = systemList.scrollHeight;
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