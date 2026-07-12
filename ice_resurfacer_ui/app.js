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
            pointRadius: 6,
            pointHoverRadius: 8
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
            x: { min: -50, max: 35, title: {display: true, text: 'Rink Length (m)', color: '#aaaaaa'}, 
            grid: { color: 'rgba(255, 255, 255, 0.1)' }, // Faint white gridlines
            ticks: { color: '#888888' }
            },
            y: { min: -25, max: 20, title: {display: true, text: 'Rink Width (m)', color: '#aaaaaa'}, 
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
                min: 0, max: 3, 
                title: {display: true, text: 'Velocity (m/s)', color: '#aaaaaa'},
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

function startSequence() {
    isRunning = true;
    document.getElementById('val-op').innerText = "Resurfacing";
    document.getElementById('val-cond').innerText = "Down";
    document.getElementById('val-nav').innerText = "Active";
    document.getElementById('val-valve').innerText = "90%";
    document.getElementById('val-auger').innerText = "15.0 rad/s";
    
    // Clear old alarms
    document.getElementById('alarm-list').innerHTML = "";
    pathData = []; // Clear old path
}

function stopSequence() {
    if (!isRunning) return; // Don't do anything if it's already stopped
    
    isRunning = false;
    document.getElementById('val-op').innerText = "Manual Override";
    document.getElementById('val-cond').innerText = "Up";
    document.getElementById('val-nav').innerText = "Aborted";
    
    // Trigger our new dynamic alarm!
    triggerAlarm("E-STOP: Operator halted sequence", "HMI Panel");
}

// ==========================================
// ROS 2 CONNECTION MANAGER
// ==========================================
const ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', () => {
    console.log('Connected to websocket server.');
    document.getElementById('val-nav').innerText = "Connected";
    document.getElementById('val-nav').style.color = "#00E676"; // Neon Green
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
    document.getElementById('val-nav').innerText = "Comms Error";
    document.getElementById('val-nav').style.color = "#ff1744"; // Red
});

ros.on('close', () => {
    console.log('Connection to websocket server closed.');
    document.getElementById('val-nav').innerText = "Offline";
    document.getElementById('val-nav').style.color = "#888";
});

const stateListener = new ROSLIB.Topic({
    ros : ros,
    name : '/mission_state',
    messageType : 'std_msgs/String'
});

stateListener.subscribe((message) => {
    // message.data contains your string (e.g., 'RESURFACING')
    document.getElementById('val-op').innerText = message.data;
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

    // 2. Update Chart.js
    let velData = velocityChart.data.datasets[0].data;
    velData.push(currentVel);
    velData.shift(); 
    velocityChart.update();
});

const odomListener = new ROSLIB.Topic({
    ros : ros,
    name : '/odometry/filtered',
    messageType : 'nav_msgs/Odometry'
});

odomListener.subscribe((message) => {
    let posX = message.pose.pose.position.x;
    let posY = message.pose.pose.position.y;

    // Update the single dot on the Position Chart
    positionChart.data.datasets[0].data = [{x: posX, y: posY}];
    positionChart.update();
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